"""Example script of moving robot joint positions."""
import argparse
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig, transform_utils
from deoxys.utils.config_utils import (get_default_controller_config,
                                       verify_controller_config)
from deoxys.utils.input_utils import input2action
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")
    args = parser.parse_args()
    return args


def compute_errors(pose_1, pose_2):

    pose_a = (
        pose_1[:3]
        + transform_utils.quat2axisangle(np.array(pose_1[3:]).flatten()).tolist()
    )
    pose_b = (
        pose_2[:3]
        + transform_utils.quat2axisangle(np.array(pose_2[3:]).flatten()).tolist()
    )
    return np.abs(np.array(pose_a) - np.array(pose_b))


def osc_move(robot_interface, controller_type, controller_cfg, target_pose, num_steps):
    target_pos, target_quat = target_pose
    target_axis_angle = transform_utils.quat2axisangle(target_quat)
    current_rot, current_pos = robot_interface.last_eef_rot_and_pos

    for _ in range(num_steps):
        current_pose = robot_interface.last_eef_pose
        current_pos = current_pose[:3, 3:]
        current_rot = current_pose[:3, :3]
        current_quat = transform_utils.mat2quat(current_rot)
        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        quat_diff = transform_utils.quat_distance(target_quat, current_quat)
        current_axis_angle = transform_utils.quat2axisangle(current_quat)
        axis_angle_diff = transform_utils.quat2axisangle(quat_diff)
        action_pos = (target_pos - current_pos).flatten() * 10
        action_axis_angle = axis_angle_diff.flatten() * 1
        action_pos = np.clip(action_pos, -1.0, 1.0)
        action_axis_angle = np.clip(action_axis_angle, -0.5, 0.5)

        action = action_pos.tolist() + action_axis_angle.tolist() + [-1.0]
        logger.info(f"Axis angle action {action_axis_angle.tolist()}")
        # print(np.round(action, 2))
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
    return action


def move_to_target_pose(
    robot_interface,
    controller_type,
    controller_cfg,
    target_delta_pose,
    num_steps,
    num_additional_steps,
    interpolation_method,
):
    while robot_interface.state_buffer_size == 0:
        logger.warn("Robot state not received")
        time.sleep(0.5)

    target_delta_pos, target_delta_axis_angle = (
        target_delta_pose[:3],
        target_delta_pose[3:],
    )
    current_ee_pose = robot_interface.last_eef_pose
    current_pos = current_ee_pose[:3, 3:]
    current_rot = current_ee_pose[:3, :3]
    current_quat = transform_utils.mat2quat(current_rot)
    current_axis_angle = transform_utils.quat2axisangle(current_quat)

    target_pos = np.array(target_delta_pos).reshape(3, 1) + current_pos

    target_axis_angle = np.array(target_delta_axis_angle) + current_axis_angle

    logger.info(f"Before conversion {target_axis_angle}")
    target_quat = transform_utils.axisangle2quat(target_axis_angle)
    target_pose = target_pos.flatten().tolist() + target_quat.flatten().tolist()

    if np.dot(target_quat, current_quat) < 0.0:
        current_quat = -current_quat
    target_axis_angle = transform_utils.quat2axisangle(target_quat)
    logger.info(f"After conversion {target_axis_angle}")
    current_axis_angle = transform_utils.quat2axisangle(current_quat)

    start_pose = current_pos.flatten().tolist() + current_quat.flatten().tolist()

    osc_move(
        robot_interface,
        controller_type,
        controller_cfg,
        (target_pos, target_quat),
        num_steps,
    )
    osc_move(
        robot_interface,
        controller_type,
        controller_cfg,
        (target_pos, target_quat),
        num_additional_steps,
    )


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_type = args.controller_type

    controller_cfg = get_default_controller_config(controller_type)

    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]

    reset_joints_to(robot_interface, reset_joint_positions)

    move_to_target_pose(
        robot_interface,
        controller_type,
        controller_cfg,
        target_delta_pose=[0.2, 0.0, 0.0, 0.0, 0.5, 0.2],
        num_steps=80,
        num_additional_steps=40,
        interpolation_method="linear",
    )

    robot_interface.close()


if __name__ == "__main__":
    main()
