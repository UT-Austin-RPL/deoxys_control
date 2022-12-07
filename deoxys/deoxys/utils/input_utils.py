import logging

import numpy as np

import deoxys.utils.transform_utils as T

logger = logging.getLogger(__name__)


def input2action(device, controller_type="OSC_POSE", robot_name="Panda", gripper_dof=1):
    state = device.get_controller_state()
    # Note: Devices output rotation with x and z flipped to account for robots starting with gripper facing down
    #       Also note that the outputted rotation is an absolute rotation, while outputted dpos is delta pos
    #       Raw delta rotations from neutral user input is captured in raw_drotation (roll, pitch, yaw)
    dpos, rotation, raw_drotation, grasp, reset = (
        state["dpos"],
        state["rotation"],
        state["raw_drotation"],
        state["grasp"],
        state["reset"],
    )

    drotation = raw_drotation[[1, 0, 2]]

    action = None

    if not reset:
        if controller_type == "OSC_POSE":
            drotation[2] = -drotation[2]
            drotation *= 75
            dpos *= 200
            drotation = drotation

            grasp = 1 if grasp else -1
            action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

        if controller_type == "OSC_YAW":
            drotation[2] = -drotation[2]
            drotation *= 75
            dpos *= 200

            grasp = 1 if grasp else -1
            action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

            # drotation = T.quat2axisangle(T.mat2quat(T.euler2mat(drotation)))
        if controller_type == "OSC_POSITION":
            drotation[:] = 0
            dpos *= 200
            grasp = 1 if grasp else -1
            action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

            # drotation = T.quat2axisangle(T.mat2quat(T.euler2mat(drotation)))

        if controller_type == "JOINT_IMPEDANCE":
            grasp = 1 if grasp else -1
            action = np.array([0.0] * 7 + [grasp] * gripper_dof)

    return action, grasp
