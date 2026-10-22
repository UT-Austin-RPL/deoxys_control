"""OSC_POSE_PRECISE: OSC_POSE's action interface, tracked through the joint impedance controller.

OSC_POSE sends `current pose + delta` to the NUC's Cartesian impedance controller, which pulls toward it with a spring
(Kp 150 N/m by default). On real arms, deltas of a few cm can produce little or no motion against joint friction.
OSC_POSE_PRECISE takes the same action, [dx, dy, dz, ax, ay, az, gripper] in the base frame scaled by `action_scale`,
and on every control() call takes one damped least squares (Levenberg-Marquardt) IK step toward `current pose + delta`,
using the geometric Jacobian at the measured end effector position. The resulting joint target goes out as a
JOINT_IMPEDANCE command, which the arm tracks.

Differences from OSC_POSE, besides tracking:
    * The translation and rotation parts of the action are each limited to norm 1 (direction kept), so
      `action_scale.translation` / `.rotation` are the largest step per control() call, in m / rad.
    * Default action_scale is much smaller than OSC_POSE's (see config/osc-pose-precise-controller.yml), because the
      step is actually taken. Actions tuned for OSC_POSE (teleop, policies) will move faster here.
    * Compliance is that of the joint impedance gains, not a Cartesian stiffness.

Requires jrl2 (forward kinematics), imported on first use.
"""

import numpy as np


class PandaJacobian:
    """Geometric Jacobian and joint limits of the Panda arm, from jrl2's forward kinematics."""

    ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]

    def __init__(self):
        from jrl2.robots import get_robot_by_name

        self._robot = get_robot_by_name("panda")
        joints = {joint.name: joint for joint in self._robot.actuated_joints}
        arm_joints = [joints[name] for name in self.ARM_JOINT_NAMES]
        self._axes = [np.asarray(joint.axis, dtype=float) for joint in arm_joints]
        self._child_links = [joint.child for joint in arm_joints]
        self.q_min = np.array([joint.limit.lower for joint in arm_joints])
        self.q_max = np.array([joint.limit.upper for joint in arm_joints])
        # The fingers don't move the arm's joint frames; any value satisfies jrl2's full configuration requirement.
        self._finger_q = {name: 0.0 for name in self._robot.actuated_joint_names if name not in self.ARM_JOINT_NAMES}

    def jacobian(self, q: np.ndarray, p_ee: np.ndarray) -> np.ndarray:
        """[6, 7] Jacobian of the point `p_ee` (base frame), position rows first, then rotation rows.

        Column i is [z_i x (p_ee - p_i); z_i] for revolute joint i with world axis z_i through point p_i. Taking p_ee
        from the measured O_T_EE keeps it consistent with whatever end effector is configured in Franka Desk.
        """
        poses = self._robot.get_all_link_poses_non_batched({**dict(zip(self.ARM_JOINT_NAMES, q)), **self._finger_q})
        J = np.zeros((6, 7))
        for i, (axis, link) in enumerate(zip(self._axes, self._child_links)):
            T = poses[link]
            z = T[:3, :3] @ axis
            J[:3, i] = np.cross(z, p_ee - T[:3, 3])
            J[3:, i] = z
        return J


def _limit_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    norm = np.linalg.norm(v)
    return v * (max_norm / norm) if norm > max_norm else v


def osc_pose_precise_joint_target(
    action: np.ndarray, q: np.ndarray, O_T_EE: np.ndarray, controller_cfg, kinematics: PandaJacobian
) -> np.ndarray:
    """Joint target [7] for one OSC_POSE_PRECISE action [6+], given the joint positions and end effector pose
    from the same state message."""
    delta_pos = _limit_norm(np.asarray(action[0:3], dtype=float), 1.0) * controller_cfg.action_scale.translation
    delta_rot = _limit_norm(np.asarray(action[3:6], dtype=float), 1.0) * controller_cfg.action_scale.rotation
    # The pose error to `current + delta` is the delta itself: OSC_POSE applies the rotation in the base frame
    # (R_goal = R_delta @ R_current), which is what the Jacobian's rotation rows (base frame angular velocity) describe.
    J = kinematics.jacobian(q, O_T_EE[:3, 3])
    error = np.concatenate([delta_pos, delta_rot])
    dq = np.linalg.solve(J.T @ J + controller_cfg.damping * np.eye(7), J.T @ error)
    dq_max = np.abs(dq).max()
    if dq_max > controller_cfg.max_dq:
        dq = dq * (controller_cfg.max_dq / dq_max)
    margin = controller_cfg.joint_limit_margin
    return np.clip(q + dq, kinematics.q_min + margin, kinematics.q_max - margin)
