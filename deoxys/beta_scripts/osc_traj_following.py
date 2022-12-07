"""
A simple script to test OSC controller in real world by trying to reach target eef poses.
"""
import argparse
import math
import os
import time

import numpy as np

np.set_printoptions(precision=3, suppress=True)

from easydict import EasyDict as edict

import deoxys
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface

# starting pose
TARGET_POS = [0.458, 0.032, 0.266]
TARGET_ROT = [
    [0.9993432241533292, 0.019870490068441337, 0.029984132035805904],
    [0.020035000112128724, -0.9997761479596957, -0.005196168052018703],
    [0.029874169621001208, 0.00579348742304635, -0.9995368685864641],
]

# a little in front of the robot
TARGET_POS = [0.564, 0.020, 0.272]
TARGET_ROT = [
    [0.9995032371565169, -0.019909445923045513, 0.02403436912962373],
    [-0.019718444808587695, -0.999762660032333, -0.008158107746105944],
    [0.024191088218237105, 0.0076801347201128365, -0.9996778452974456],
]

# a little left twist in front of the robot
TARGET_POS = [0.496, -0.075, 0.222]
TARGET_ROT = [
    [0.8562071466215979, -0.5138655676566113, 0.05321937507004706],
    [-0.5165554492241982, -0.850003931710708, 0.10317332891054326],
    [-0.007780543174911574, -0.11582849975068715, -0.9932386230502067],
]

# a little right twist in front of the robot
TARGET_POS = [0.510, 0.180, 0.223]
TARGET_ROT = [
    [0.9241596173608245, 0.38124282834155593, 0.02374163497184082],
    [0.3819250292698003, -0.9211696470599203, -0.07456947976816128],
    [-0.006559005867136314, 0.07798162652087287, -0.996933160977189],
]

# far bottom left - maintain rot
TARGET_POS = [0.664, -0.247, 0.141]
TARGET_ROT = [
    [0.9993432241533292, 0.019870490068441337, 0.029984132035805904],
    [0.020035000112128724, -0.9997761479596957, -0.005196168052018703],
    [0.029874169621001208, 0.00579348742304635, -0.9995368685864641],
]

# far bottom left - horizontal twist
TARGET_POS = [0.664, -0.247, 0.141]
TARGET_ROT = [
    [0.6227273317758799, -0.7635326683965938, 0.17090887581307032],
    [-0.7605314318130421, -0.6419965494963049, -0.09702216136348514],
    [0.1838024983097502, -0.06956322036265966, -0.9804982694388963],
]

# far bottom left - vertical twist
TARGET_POS = [0.664, -0.247, 0.141]
TARGET_ROT = [
    [0.877546930468378, -0.18049967872278125, 0.4442024275577914],
    [-0.09735233072507901, -0.9742045402581938, -0.2035425802915927],
    [0.46948339206955014, 0.13537402491801145, -0.8724990666814524],
]

# far bottom left - horizontal + vertical twist - good test case rotation (hard)
TARGET_POS = [0.664, -0.247, 0.141]
TARGET_ROT = [
    [0.41218748609555816, -0.6951155569261588, 0.5889849443657237],
    [-0.6928120108154153, -0.6589602725541662, -0.29285708974474306],
    [0.591687198507229, -0.28734381603887565, -0.7532140866292563],
]

# far bottom right - maintain rot
TARGET_POS = [0.682, 0.141, 0.123]
TARGET_ROT = [
    [0.9993432241533292, 0.019870490068441337, 0.029984132035805904],
    [0.020035000112128724, -0.9997761479596957, -0.005196168052018703],
    [0.029874169621001208, 0.00579348742304635, -0.9995368685864641],
]

# far bottom center - maintain rot
TARGET_POS = [0.680, -0.011, 0.184]
TARGET_ROT = [
    [0.9993432241533292, 0.019870490068441337, 0.029984132035805904],
    [0.020035000112128724, -0.9997761479596957, -0.005196168052018703],
    [0.029874169621001208, 0.00579348742304635, -0.9995368685864641],
]

# far bottom right - good test case rotation (hard)
TARGET_POS = [0.648, 0.307, 0.164]
TARGET_ROT = [
    [0.6418107664433936, 0.5567554825361436, 0.5273408515442645],
    [0.730866388419942, -0.6522840647209083, -0.20085154912587905],
    [0.23215083298696704, 0.5143243903202767, -0.8255726991042672],
]

#############################
# SET REAL POSE TO USE HERE #
#############################

# far bottom center - maintain rot
TARGET_POS = [0.680, -0.011, 0.184]
TARGET_ROT = [
    [0.9993432241533292, 0.019870490068441337, 0.029984132035805904],
    [0.020035000112128724, -0.9997761479596957, -0.005196168052018703],
    [0.029874169621001208, 0.00579348742304635, -0.9995368685864641],
]

# number of interpolation steps
NUM_T_INTERP = 25

# number of steps with fixed target pose after interpolation
NUM_T_FIXED = 25

# config yaml for deoxys
DEOXYS_CONFIG_YAML = os.path.join(deoxys.__path__[0], "../config/alice.yml")

# gains dict
CONTROLLER_CFG_DICT = edict(
    dict(
        Kp=dict(
            # translation=130.0,
            # rotation=70.0,
            translation=150.0,
            rotation=250.0,
            # translation=150.0,
            # rotation=150.0,
        ),
    )
)

# scale for action commands
MAX_DPOS = np.array([0.08, 0.08, 0.08])
MAX_DROT = np.array([0.5, 0.5, 0.5])


def make_env():
    """
    Helper function to make real robot environment.
    """
    robot_interface = FrankaInterface(
        general_cfg_file=os.path.join(config_root, DEOXYS_CONFIG_YAML),
        control_freq=20,
        state_freq=100,
        control_timeout=1.0,
        has_gripper=True,
        use_visualizer=False,
        debug=False,
    )

    return robot_interface


def reset_env(robot_interface):
    """
    Use this function to reset the robot state to a known configuration before trying to reach
    a target pose.
    """
    robot_interface.clear_buffer()

    print("restarting the robot interface")

    # Code below based on https://github.com/UT-Austin-RPL/robot_infra/blob/master/deoxys/examples/reset_robot_joints.py

    # Golden resetting joints
    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]
    action = reset_joint_positions + [-1.0]

    while True:
        if len(robot_interface._state_buffer) > 0:
            if (
                np.max(
                    np.abs(
                        np.array(robot_interface._state_buffer[-1].q)
                        - np.array(reset_joint_positions)
                    )
                )
                < 1e-2
            ):
                break

        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=action,
            controller_cfg=CONTROLLER_CFG_DICT,
        )

    # We added this sleep here to give the C++ controller time to reset from joint control mode to no control mode
    # to prevent some issues.
    time.sleep(1.1)


def do_osc_action(robot_interface, action, controller_cfg_dict=None):
    """Send normalized delta pose action to robot."""
    if controller_cfg_dict is None:
        controller_cfg_dict = CONTROLLER_CFG_DICT

    robot_interface.control(
        controller_type="OSC_POSE",
        action=action,
        controller_cfg=controller_cfg_dict,
    )


def do_osc_absolute_pose_action(
    robot_interface, target_pos, target_rot, controller_cfg_dict=None
):
    """Convert absolute pose action to delta pose action and then send to robot."""

    # read current eef pose
    start_pos, start_rot = get_eef_pose(robot_interface)

    # normalize delta position action
    delta_position = target_pos - start_pos
    delta_position = np.clip(delta_position / MAX_DPOS, -1.0, 1.0)

    # normalize delta rotation action
    delta_rot_mat = target_rot.dot(start_rot.T)
    delta_rot_quat = mat2quat(delta_rot_mat)
    delta_rot_aa = quat2axisangle(delta_rot_quat)
    delta_rotation = delta_rot_aa[0] * delta_rot_aa[1]
    delta_rotation = np.clip(delta_rotation / MAX_DROT, -1.0, 1.0)
    pose_action = np.concatenate([delta_position, delta_rotation])

    # assume gripper is open for now
    action = np.concatenate([pose_action, [-1.0]])
    # print("action: {}".format(action))
    # print("target_pos: {}".format(target_pos))
    # print("curr_pos: {}".format(start_pos))
    do_osc_action(
        robot_interface=robot_interface,
        action=action,
        controller_cfg_dict=controller_cfg_dict,
    )


def get_eef_pose(robot_interface):
    """Get current robot eef pose"""
    last_robot_state = robot_interface._state_buffer[-1]
    ee_pose = np.array(last_robot_state.O_T_EE).reshape((4, 4)).T
    ee_pos = ee_pose[:3, 3]
    ee_rot = ee_pose[:3, :3]
    return ee_pos, ee_rot


def get_target_pose(robot_interface):
    """
    Use this function to get a target OSC pose to try and reach.
    """
    return np.array(TARGET_POS), np.array(TARGET_ROT).reshape(3, 3)


def print_debug(x):
    """Helpful print function."""
    print("")
    print("*" * 50)
    print(x)
    print("*" * 50)
    print("")


def quat2mat(quaternion):
    """
    Converts given quaternion (x, y, z, w) to matrix.

    Args:
        quaternion: vec4 float angles

    Returns:
        3x3 rotation matrix
    """

    # awkward semantics for use with numba
    inds = np.array([3, 0, 1, 2])
    q = np.asarray(quaternion).copy().astype(np.float32)[inds]

    n = np.dot(q, q)
    if n < (np.finfo(float).eps * 4.0):
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q2 = np.outer(q, q)
    return np.array(
        [
            [1.0 - q2[2, 2] - q2[3, 3], q2[1, 2] - q2[3, 0], q2[1, 3] + q2[2, 0]],
            [q2[1, 2] + q2[3, 0], 1.0 - q2[1, 1] - q2[3, 3], q2[2, 3] - q2[1, 0]],
            [q2[1, 3] - q2[2, 0], q2[2, 3] + q2[1, 0], 1.0 - q2[1, 1] - q2[2, 2]],
        ]
    )


def mat2quat(rmat, precise=False):
    """
    Converts given rotation matrix to quaternion.

    Args:
        rmat: 3x3 rotation matrix
        precise: If isprecise is True, the input matrix is assumed to be a precise
             rotation matrix and a faster algorithm is used.

    Returns:
        vec4 float quaternion angles
    """
    M = np.asarray(rmat).astype(np.float32)[:3, :3]
    if precise:
        # This code uses a modification of the algorithm described in:
        # https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
        # which is itself based on the method described here:
        # http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        # Altered to work with the column vector convention instead of row vectors
        m = (
            M.conj().transpose()
        )  # This method assumes row-vector and postmultiplication of that vector
        if m[2, 2] < 0:
            if m[0, 0] > m[1, 1]:
                t = 1 + m[0, 0] - m[1, 1] - m[2, 2]
                q = [m[1, 2] - m[2, 1], t, m[0, 1] + m[1, 0], m[2, 0] + m[0, 2]]
            else:
                t = 1 - m[0, 0] + m[1, 1] - m[2, 2]
                q = [m[2, 0] - m[0, 2], m[0, 1] + m[1, 0], t, m[1, 2] + m[2, 1]]
        else:
            if m[0, 0] < -m[1, 1]:
                t = 1 - m[0, 0] - m[1, 1] + m[2, 2]
                q = [m[0, 1] - m[1, 0], m[2, 0] + m[0, 2], m[1, 2] + m[2, 1], t]
            else:
                t = 1 + m[0, 0] + m[1, 1] + m[2, 2]
                q = [t, m[1, 2] - m[2, 1], m[2, 0] - m[0, 2], m[0, 1] - m[1, 0]]
        q = np.array(q)
        q *= 0.5 / np.sqrt(t)
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array(
            [
                [m00 - m11 - m22, np.float32(0.0), np.float32(0.0), np.float32(0.0)],
                [m01 + m10, m11 - m00 - m22, np.float32(0.0), np.float32(0.0)],
                [m02 + m20, m12 + m21, m22 - m00 - m11, np.float32(0.0)],
                [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
            ]
        )
        K /= 3.0
        # quaternion is Eigen vector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        inds = np.array([3, 0, 1, 2])
        q1 = V[inds, np.argmax(w)]
    if q1[0] < 0.0:
        np.negative(q1, q1)
    inds = np.array([1, 2, 3, 0])
    return q1[inds]


def quat2axisangle(quat):
    """
    Converts (x, y, z, w) quaternion to axis-angle format.
    Returns a unit vector direction and an angle.
    """

    # conversion from axis-angle to quaternion:
    #   qw = cos(theta / 2); qx, qy, qz = u * sin(theta / 2)

    # normalize qx, qy, qz by sqrt(qx^2 + qy^2 + qz^2) = sqrt(1 - qw^2)
    # to extract the unit vector

    # clipping for scalar with if-else is orders of magnitude faster than numpy
    if quat[3] > 1.0:
        quat[3] = 1.0
    elif quat[3] < -1.0:
        quat[3] = -1.0

    den = np.sqrt(1.0 - quat[3] * quat[3])
    if math.isclose(den, 0.0):
        # This is (close to) a zero degree rotation, immediately return
        return np.zeros(3), 0.0

    # convert qw to theta
    theta = 2.0 * math.acos(quat[3])

    return quat[:3] / den, 2.0 * math.acos(quat[3])


def interpolate_poses(pos1, rot1, pos2, rot2, num_steps):
    """
    Linear interpolation between two poses.

    Args:
        pos1 (np.array): np array of shape (3,) for first position
        rot1 (np.array): np array of shape (3, 3) for first rotation
        pos2 (np.array): np array of shape (3,) for second position
        rot2 (np.array): np array of shape (3, 3) for second rotation
        num_steps (int): specifies the number of desired interpolated points (not including
            the start and end points). Passing 0 corresponds to no interpolation.

    Returns:
        pos_steps (np.array): array of shape (N + 2, 3) corresponding to the interpolated position path, where N is @num_steps
        rot_steps (np.array): array of shape (N + 2, 3, 3) corresponding to the interpolated rotation path, where N is @num_steps
        num_steps (int): the number of interpolated points (N) in the path
    """

    if num_steps == 0:
        # skip interpolation
        return (
            np.concatenate([pos1[None], pos2[None]], axis=0),
            np.concatenate([rot1[None], rot2[None]], axis=0),
            num_steps,
        )

    delta_pos = pos2 - pos1
    num_steps += 1  # include starting pose
    assert num_steps >= 2

    # linear interpolation of positions
    pos_step_size = delta_pos / num_steps
    grid = np.arange(num_steps).astype(np.float64)
    pos_steps = np.array([pos1 + grid[i] * pos_step_size for i in range(num_steps)])

    # add in endpoint
    pos_steps = np.concatenate([pos_steps, pos2[None]], axis=0)

    # interpolate the rotations too
    rot_steps = interpolate_rotations(R1=rot1, R2=rot2, num_steps=num_steps)

    return pos_steps, rot_steps, num_steps - 1


def interpolate_rotations(R1, R2, num_steps):
    """
    Interpolate between 2 rotation matrices.
    """
    q1 = mat2quat(R1)
    q2 = mat2quat(R2)
    rot_steps = np.array(
        [
            quat2mat(quat_slerp(q1, q2, tau=(float(i) / num_steps)))
            for i in range(num_steps)
        ]
    )

    # add in endpoint
    rot_steps = np.concatenate([rot_steps, R2[None]], axis=0)

    return rot_steps


def quat_slerp(q1, q2, tau):
    """
    Adapted from robosuite.
    """
    if tau == 0.0:
        return q1
    elif tau == 1.0:
        return q2
    d = np.dot(q1, q2)
    if abs(abs(d) - 1.0) < np.finfo(float).eps * 4.0:
        return q1
    if d < 0.0:
        # invert rotation
        d = -d
        q2 *= -1.0
    angle = math.acos(np.clip(d, -1, 1))
    if abs(angle) < np.finfo(float).eps * 4.0:
        return q1
    isin = 1.0 / math.sin(angle)
    q1 = q1 * math.sin((1.0 - tau) * angle) * isin
    q2 = q2 * math.sin(tau * angle) * isin
    q1 = q1 + q2
    return q1


def try_and_reach_pose(robot_interface):
    """
    Function that runs a trial by resetting the environment, getting a new
    target pose to reach, and trying to reach it.
    """

    # reset environment and get starting and target poses
    reset_env(robot_interface)
    start_pos, start_rot = get_eef_pose(robot_interface)
    target_pos, target_rot = get_target_pose(robot_interface)

    rot_err_mat = target_rot.dot(start_rot.T)
    _, rot_err_angle = quat2axisangle(mat2quat(rot_err_mat))
    rot_err_angle = 180.0 * rot_err_angle / np.pi

    print_debug("Starting pose\npos: {}\nrot: {}".format(start_pos, start_rot))
    print_debug("Target pose\npos: {}\nrot: {}".format(target_pos, target_rot))
    print_debug("Rotation diff (deg): {}".format(rot_err_angle))

    if NUM_T_INTERP > 0:
        # get interpolated path
        pos_targets_interp, rot_targets_interp, _ = interpolate_poses(
            pos1=start_pos,
            rot1=start_rot,
            pos2=target_pos,
            rot2=target_rot,
            num_steps=NUM_T_INTERP,
        )

        # try to follow interpolated path
        for t in range(pos_targets_interp.shape[0]):
            do_osc_absolute_pose_action(
                robot_interface,
                target_pos=pos_targets_interp[t],
                target_rot=rot_targets_interp[t],
                controller_cfg_dict=CONTROLLER_CFG_DICT,
            )

    if NUM_T_FIXED > 0:
        # additional steps with fixed endpoint target
        for t in range(NUM_T_FIXED):
            do_osc_absolute_pose_action(
                robot_interface,
                target_pos=target_pos,
                target_rot=target_rot,
                controller_cfg_dict=CONTROLLER_CFG_DICT,
            )

    new_pos, new_rot = get_eef_pose(robot_interface)
    print_debug("Final pose\npos: {}\nrot: {}".format(new_pos, new_rot))

    # compute pose error
    pos_err = np.abs(new_pos - target_pos)
    rot_err_mat = new_rot.dot(target_rot.T)
    _, rot_err_angle = quat2axisangle(mat2quat(rot_err_mat))
    rot_err_angle = 180.0 * rot_err_angle / np.pi
    print_debug(
        "Pose Error\nabs pos err: {}\ndelta rot angle (deg): {}".format(
            pos_err, rot_err_angle
        )
    )


def reset_and_wait():
    """
    Reset the robot and set impedance low, allowing user to drag robot around. Also
    print robot eef pose repeatedly. This function is useful to get suitable target
    OSC poses to try and reach.
    """

    # set gains to low impedance for easy manual movement
    controller_cfg_dict = edict(
        dict(
            Kp=dict(
                translation=50.0,
                rotation=50.0,
            ),
        )
    )

    robot_interface = make_env()

    # reset arm
    reset_env(robot_interface)

    zero_action = np.zeros(7)
    zero_action[-1] = -1.0

    cnt = 0
    while True:
        # maintain zero control
        do_osc_action(
            robot_interface=robot_interface,
            action=zero_action,
            controller_cfg_dict=controller_cfg_dict,
        )

        if cnt % 100 == 0:
            # print eef pose every 5 seconds
            eef_pos, eef_rot = get_eef_pose(robot_interface)
            print("TARGET_POS = [{:.3f}, {:.3f}, {:.3f}]".format(*eef_pos))
            print(
                "TARGET_ROT = [\n    [{}, {}, {}],\n    [{}, {}, {}],\n    [{}, {}, {}],\n]".format(
                    *eef_rot.reshape(-1)
                )
            )
        cnt += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--reset",
        action="store_true",
        help="reset the robot and allow user to drag robot arm to read off and record suitable target poses",
    )
    args = parser.parse_args()

    if args.reset:
        reset_and_wait()
    else:
        robot_interface = make_env()
        try_and_reach_pose(robot_interface)
