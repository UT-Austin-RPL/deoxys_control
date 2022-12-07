import pathlib
from abc import ABC, abstractmethod

import numpy as np
import pybullet

FILE_PATH = pathlib.Path(__file__).parent.absolute()


def visualizer_factory(backend, *args, **kwargs):
    if backend == "pybullet":
        return PybulletVisualizer(*args, **kwargs)
    else:
        raise NotImplementedError


class Visualizer:
    def __init__(self):
        pass

    @abstractmethod
    def render(self):
        raise NotImplementedError


class PybulletVisualizer(Visualizer):
    def __init__(self):
        super().__init__()
        self._uid = pybullet.connect(pybullet.GUI)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 1)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

        position = [0.0, 0.0, 0.0]
        quaternion = [0.0, 0.0, 0.0, 1.0]
        scale = 1.0

        pybullet.loadURDF(
            fileName=str(FILE_PATH) + "/robot_models/planes/plane_ceramic.urdf",
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            globalScaling=1.0,
            useFixedBase=True,
            physicsClientId=self._uid,
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )

        self.robot_uid = pybullet.loadURDF(
            fileName=str(FILE_PATH) + "/robot_models/panda/panda_marker.urdf",
            basePosition=position,
            baseOrientation=quaternion,
            globalScaling=scale,
            useFixedBase=True,
            physicsClientId=self._uid,
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=1.4,
            cameraYaw=45,
            cameraPitch=-45,
            cameraTargetPosition=np.array(position) + np.array([0.0, 0.0, 0.3]),
            physicsClientId=self._uid,
        )

        self.num_arm_joints = 7

        # The marker is the last link of the URDF file
        self.marker_ind = (
            pybullet.getNumJoints(self.robot_uid, physicsClientId=self._uid) - 1
        )

    def render(self):
        # Render
        pybullet.stepSimulation(physicsClientId=self._uid)

    def update(self, joint_positions, vis_gripper=False):
        # Update states
        num_robot_joints = 0
        if not vis_gripper:
            assert len(joint_positions) == self.num_arm_joints
            num_robot_joints = self.num_arm_joints

        for joint_ind in range(self.num_arm_joints):
            pybullet.resetJointState(
                bodyUniqueId=self.robot_uid,
                jointIndex=joint_ind,
                targetValue=joint_positions[joint_ind],
                physicsClientId=self._uid,
            )

    def get_marker_pose(self):
        marker_link_state = pybullet.getLinkState(
            bodyUniqueId=self.robot_uid,
            jointIndex=i,
            targetValue=self.marker_ind,
            physicsClientId=self._uid,
        )
        marker_pos, marker_quat = marker_link_state[0], marker_link_state[1]
        return (marker_pos, marker_quat)
