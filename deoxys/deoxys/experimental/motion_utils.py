"""
This is an experimental file where we have some standard abstractions for certain manipulation behaviors. This part will be made standard once we've tested.
"""
import time
from typing import Union

import numpy as np

from deoxys.utils.config_utils import (get_default_controller_config,
                                       verify_controller_config)

# Joint space motion abstractions


def reset_joints_to(
    robot_interface,
    start_joint_pos: Union[list, np.ndarray],
    controller_cfg: dict = None,
    timeout=7,
    gripper_open=False,
):
    assert type(start_joint_pos) is list or type(start_joint_pos) is np.ndarray
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="JOINT_POSITION")
    else:
        assert controller_cfg["controller_type"] == "JOINT_POSITION", (
            "This function is only for JOINT POSITION mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    if gripper_open:
        gripper_action = -1
    else:
        gripper_action = 1
    if type(start_joint_pos) is list:
        action = start_joint_pos + [gripper_action]
    else:
        action = start_joint_pos.tolist() + [gripper_action]
    start_time = time.time()
    while True:
        if (
            robot_interface.received_states
            and robot_interface.check_nonzero_configuration()
        ):
            if (
                np.max(
                    np.abs(np.array(robot_interface.last_q) - np.array(start_joint_pos))
                )
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=action,
            controller_cfg=controller_cfg,
        )
        end_time = time.time()

        # Add timeout
        if end_time - start_time > timeout:
            break
    robot_interface.close()
    return True


def joint_interpolation_traj(
    start_q, end_q, num_steps=100, traj_interpolator_type="min_jerk"
):
    assert traj_interpolator_type in ["min_jerk", "linear"]

    traj = []

    if traj_interpolator_type == "min_jerk":
        for i in range(0, num_steps + 1):
            t = float(i) * (1 / num_steps)
            transformed_step_size = 10.0 * (t**3) - 15 * (t**4) + 6 * (t**5)
            traj.append(start_q + transformed_step_size * (end_q - start_q))
        traj = np.array(traj)
    elif traj_interpolator_type == "linear":
        step_size = (end_q - start_q) / float(num_steps)
        grid = np.arange(num_steps).astype(np.float64)
        traj = np.array([start_q + grid[i] * step_size for i in range(num_steps)])

        # add endpoint
        traj = np.concatenate([traj, end_q[None]], axis=0)
    return traj


def follow_joint_traj(
    robot_interface,
    joint_traj: list,
    num_addition_steps=30,
    controller_cfg: dict = None,
    gripper_close=True,
):
    """This is a simple function to follow a given trajectory in joint space.

    Args:
        robot_interface (FrankaInterface): _description_
        joint_traj (list): _description_
        num_addition_steps (int, optional): _description_. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
    Returns:
        joint_pos_history (list): a list of recorded joint positions
        action_history (list): a list of recorded action commands
    """

    if controller_cfg is None:
        controller_cfg = get_default_controller_config(
            controller_type="JOINT_IMPEDANCE"
        )
    else:
        assert controller_cfg["controller_type"] == "JOINT_IMPEDANCE", (
            "This function is only for JOINT IMPEDANCE mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)

    prev_action = np.array([0.0] * 8)
    joint_pos_history = []
    action_history = []

    assert (
        robot_interface.last_q is not None
        and robot_interface.check_nonzero_configuration()
    )

    for target_joint_pos in joint_traj:
        assert len(target_joint_pos) >= 7
        if type(target_joint_pos) is np.ndarray:
            action = target_joint_pos.tolist()
        else:
            action = target_joint_pos
        if len(action) == 7:
            if gripper_close:
                action = action + [1.0]
            else:
                action = action + [-1.0]
        current_joint_pos = np.array(robot_interface.last_q)
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )
        joint_pos_history.append(current_joint_pos.flatten().tolist())
        action_history.append(prev_action.tolist())
        prev_action = np.array(action)

    for i in range(num_addition_steps):
        current_joint_pos = np.array(robot_interface.last_q)
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )
        joint_pos_history.append(current_joint_pos.flatten().tolist())
        action_history.append(prev_action.tolist())
        prev_action = np.array(action)

    return joint_pos_history, action_history


def position_only_gripper_move_to(
    robot_interface, target_pos, num_steps=100, controller_cfg: dict = None, grasp=False
):
    """_summary_

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        target_pos (np.array or list): target xyz location
        num_steps (int, optional): number of steps to control. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
        grasp (bool, optional): close the gripper if set to True. Defaults to False.
    Return:
        eef_pos_history (list): a list of recorded end effector positions
        action_history (list): a list of recorded action commands
    """
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="OSC_POSITION")
    else:
        assert controller_cfg["controller_type"] == "OSC_POSITION", (
            "This function is only for OSC_POSITION mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)
    eef_pos_history = []
    action_history = []

    current_pos = None
    while current_pos is None:
        _, current_pos = robot_interface.last_eef_rot_and_pos

    prev_action = np.array([0.0] * 6 + [int(grasp) * 2 - 1])
    for i in range(num_steps):
        _, current_pos = robot_interface.last_eef_rot_and_pos
        action = np.array([0.0] * 6 + [int(grasp) * 2 - 1])
        action[:3] = (target_pos - current_pos).flatten() * 10
        robot_interface.control(
            controller_type="OSC_POSITION", action=action, controller_cfg=controller_cfg
        )
        eef_pos_history.append(current_pos.flatten().tolist())
        action_history.append(prev_action)
        prev_action = np.array(action)
    return eef_pos_history, action_history


def position_only_gripper_move_by(
    robot_interface, delta_pos, num_steps=100, controller_cfg: dict = None, grasp=True
):
    """_summary_

    Args:
        robot_interface (FrankaInterface): the python interface for robot control
        target_pos (np.array or list): target xyz location
        num_steps (int, optional): number of steps to control. Defaults to 100.
        controller_cfg (dict, optional): controller configurations. Defaults to None.
        grasp (bool, optional): close the gripper if set to True. Defaults to False.
    Return:
        eef_pos_history (list): a list of recorded end effector positions
        action_history (list): a list of recorded action commands
    """
    current_pos = None
    while current_pos is None:
        _, current_pos = robot_interface.last_eef_rot_and_pos

    delta_pos = np.array(delta_pos).reshape(3, 1)
    assert delta_pos.shape == current_pos.shape
    target_pos = current_pos + delta_pos
    return position_only_gripper_move_to(
        robot_interface,
        target_pos,
        num_steps=num_steps,
        controller_cfg=controller_cfg,
        grasp=grasp,
    )
