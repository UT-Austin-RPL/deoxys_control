"""
Test script for joint impedance controller - just try to reach a nearby joint position by following
an interpolated path.
"""
import argparse
import json
import os
import pickle
import threading
import time
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np

import deoxys.utils.transform_utils as T
from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--controller-cfg", type=str, default="osc-pose-controller.yml")
    parser.add_argument("--folder", type=Path, default="benchmark_results/osc_pose")

    args = parser.parse_args()
    return args


def reset_to_joints(robot_interface, controller_cfg, reset_joint_positions):
    controller_type = "JOINT_POSITION"

    action = reset_joint_positions + [-1.0]
    start_time = time.time()
    while True:
        if len(robot_interface._state_buffer) > 0:
            if (
                np.max(
                    np.abs(
                        np.array(robot_interface._state_buffer[-1].q)
                        - np.array(reset_joint_positions)
                    )
                )
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        end_time = time.time()
        if end_time - start_time > 5.0:
            break

    # wait for at least 1 second before returning to avoid joint impedance issue
    time.sleep(1)


def interpolate(start_jpos, end_jpos, num_steps=50, interpolation_method="linear"):
    """
    Helper function to interpolate from a start joint pos to an end joint pos.
    """

    # include starting pose
    num_steps += 1

    start = np.array(start_jpos).reshape(-1)
    end = np.array(end_jpos).reshape(-1)

    if interpolation_method == "linear":
        step_size = (end - start) / float(num_steps)
        grid = np.arange(num_steps).astype(np.float64)
        steps = np.array([start + grid[i] * step_size for i in range(num_steps)])

        # add endpoint
        steps = np.concatenate([steps, end[None]], axis=0)
    elif interpolation_method == "min_jerk":
        steps = []
        for i in range(0, num_steps + 1):
            t = float(i) * (1 / num_steps)
            transformed_step_size = 10.0 * (t**3) - 15 * (t**4) + 6 * (t**5)
            steps.append(start + transformed_step_size * (end - start))
        steps = np.array(steps)
    return steps


def move_to_target_pose(
    robot_interface,
    controller_cfg,
    target_delta_pose,
    num_steps,
    num_additional_steps,
    interpolation_method,
):
    """
    Use joint impedance controller to move to a new joint position using interpolation.
    """
    while True:
        if len(robot_interface._state_buffer) > 0:
            if not robot_interface.check_nonzero_configuration():
                print(
                    len(robot_interface._state_buffer),
                    np.array(robot_interface._state_buffer[-1].O_T_EE),
                )
                continue
            else:
                break
    target_delta_pos, target_delta_axis_angle = (
        target_delta_pose[:3],
        target_delta_pose[3:],
    )
    current_ee_pose = robot_interface.last_eef_pose
    current_pos = current_ee_pose[:3, 3:]
    current_rot = current_ee_pose[:3, :3]
    current_quat = T.mat2quat(current_rot)
    current_axis_angle = T.quat2axisangle(current_quat)

    target_pos = np.array(target_delta_pos).reshape(3, 1) + current_pos

    target_axis_angle = np.array(target_delta_axis_angle) + current_axis_angle
    target_quat = T.axisangle2quat(target_axis_angle)
    target_pose = target_pos.flatten().tolist() + target_quat.flatten().tolist()

    if np.dot(target_quat, current_quat) < 0.0:
        current_quat = -current_quat
    target_axis_angle = T.quat2axisangle(target_quat)
    current_axis_angle = T.quat2axisangle(current_quat)

    start_pose = current_pos.flatten().tolist() + current_quat.flatten().tolist()

    action_history = []
    state_history = []

    def osc_move(target_pose, num_steps):
        target_pos, target_quat = target_pose
        current_rot, current_pos = robot_interface.last_eef_rot_and_pos

        prev_action = np.array([0.0] * 6)
        for _ in range(num_steps):
            current_pose = (
                np.array(robot_interface._state_buffer[-1].O_T_EE)
                .reshape(4, 4)
                .transpose()
            )
            current_pos = current_pose[:3, 3:]
            current_rot = current_pose[:3, :3]
            current_quat = T.mat2quat(current_rot)
            if np.dot(target_quat, current_quat) < 0.0:
                current_quat = -current_quat
            quat_diff = T.quat_distance(target_quat, current_quat)
            current_axis_angle = T.quat2axisangle(current_quat)
            axis_angle_diff = T.quat2axisangle(quat_diff)
            # print(np.round(current_pos.flatten(), 2))
            action_pos = (target_pos - current_pos).flatten() * 10
            action_axis_angle = axis_angle_diff.flatten() * 1
            # print(axis_angle_diff)
            action_pos = np.clip(action_pos, -1.0, 1.0)
            action_axis_angle = np.clip(
                action_axis_angle, -0.5, 0.5
            )  # * np.sin(_ / num_iters * np.pi)

            action = action_pos.tolist() + action_axis_angle.tolist() + [-1.0]
            # print(np.round(action, 2))
            robot_interface.control(
                controller_type="OSC_POSE", action=action, controller_cfg=controller_cfg
            )
            state_history.append(
                current_pos.flatten().tolist() + current_quat.flatten().tolist()
            )
            action_history.append(prev_action)
            prev_action = np.array(action)[:-1]
        return action

    action = osc_move((target_pos, target_quat), num_steps)
    reached_ee_rot, reached_ee_pos = robot_interface.last_eef_rot_and_pos
    reached_ee_quat = T.mat2quat(reached_ee_rot)
    if np.dot(target_quat, reached_ee_quat) < 0.0:
        reached_ee_quat = -reached_ee_quat

    action = osc_move((target_pos, target_quat), num_additional_steps)
    reached_ee_pose = (
        reached_ee_pos.flatten().tolist() + reached_ee_quat.flatten().tolist()
    )

    for _ in range(30):
        current_rot, current_pos = robot_interface.last_eef_rot_and_pos
        current_quat = T.mat2quat(current_rot)
        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        action_history.append(np.array(action)[:-1])
        state_history.append(
            current_pos.flatten().tolist() + current_quat.flatten().tolist()
        )
    current_ee_rot, current_ee_pos = robot_interface.last_eef_rot_and_pos
    current_ee_quat = T.mat2quat(current_ee_rot)
    if np.dot(target_quat, current_ee_quat) < 0.0:
        current_ee_quat = -current_ee_quat
    current_ee_pose = (
        current_ee_pos.flatten().tolist() + current_ee_quat.flatten().tolist()
    )

    action_history = np.array(action_history)
    state_history = np.array(state_history)
    # print final error
    def compute_errors(pose_1, pose_2):

        pose_a = pose_1[:3] + T.quat2axisangle(np.array(pose_1[3:]).flatten()).tolist()
        pose_b = pose_2[:3] + T.quat2axisangle(np.array(pose_2[3:]).flatten()).tolist()
        return np.abs(np.array(pose_a) - np.array(pose_b))

    print("absolute error: {}".format(compute_errors(reached_ee_pose, target_pose)))
    print("absolute error: {}".format(compute_errors(current_ee_pose, target_pose)))

    return {
        "target_pose": target_pose,
        "start_pose": start_pose,
        "reached_pose": reached_ee_pose,
        "reached_pose_in_addtional_steps": current_ee_pose,
        "state_history": state_history,
        "action_history": action_history,
    }


def main():
    args = parse_args()

    os.makedirs(args.folder, exist_ok=True)

    # We test the following configurations, ranging from high-level trajectory planning to low level control hyperparameters
    # number of interplation steps
    num_exp = 1
    num_steps_list = [40, 80, 120]
    num_additional_steps = 40
    traj_interpolation_fraction_list = [0.3]  # [0.3, 0.7, 1.0]
    K_list = [
        # (150.0, 250.0),
        ([150.0, 150.0, 60.0], 250.0),
        ([60.0, 150.0, 150.0], 250.0),
    ]
    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]
    residual_mass_vec_list = [
        [0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.5],
        # [0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.3],
        # [0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1],
    ]

    osc_configuration_tuples = [
        (reset_joint_positions, [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Move forward
        # (reset_joint_positions, [0.0, 0.2, 0.0, 0.0, 0.0, 0.0]),  # Move left
        # (reset_joint_positions, [0.0, -0.2, 0.0, 0.0, 0.0, 0.0]),  # Move right
        (reset_joint_positions, [-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),  # Move backward
        (reset_joint_positions, [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]),  # Move up
        (reset_joint_positions, [0.0, 0.0, -0.2, 0.0, 0.0, 0.0]),  # Move down
        # (reset_joint_positions, [0.0, 0.0, 0.0, 0.8, 0.0, 0.0]),  # rotate x axis
        # (reset_joint_positions, [0.0, 0.0, 0.0, 0.0, 0.8, 0.0]),  # rotate y axis
        # (reset_joint_positions, [0.0, 0.0, 0.0, 0.0, 0.0, 0.8]),  # rotate z axis
        # (reset_joint_positions, [0.1, 0.1, 0.1, 0.3, 0.3, 0.3]),  # mix rotate + move 1
        # (
        #     reset_joint_positions,
        #     [0.1, -0.1, 0.1, -0.3, -0.3, -0.3],
        # ),  # mix rotate + move 2
    ]

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )

    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()
    datafile_name = f"{args.folder}/data.hdf5"
    with h5py.File(datafile_name, "w") as datafile:
        data_grp = datafile.create_group("data")
        for exp_id in range(num_exp):
            count = 0
            for num_steps in num_steps_list:
                for Kp_translation, Kp_rotation in K_list:
                    print("kp position: ", Kp_translation)
                    print("kp rotation: ", Kp_rotation)
                    for interpolation_method in ["linear"]:
                        for (
                            traj_interpolation_fraction
                        ) in traj_interpolation_fraction_list:
                            for residual_mass_vec in residual_mass_vec_list:
                                print(f"Residual mass vec is: {residual_mass_vec}")
                                for (
                                    start_q,
                                    target_delta_pose,
                                ) in osc_configuration_tuples:
                                    count += 1
                                    ep_grp = data_grp.create_group(f"{exp_id}_{count}")
                                    controller_cfg.residual_mass_vec = residual_mass_vec
                                    controller_cfg.Kp.translation = Kp_translation
                                    controller_cfg.Kp.rotation = Kp_rotation
                                    # loop through each set of hyperparameter
                                    # loop through each joint configuration and record data
                                    reset_to_joints(
                                        robot_interface, controller_cfg, start_q
                                    )
                                    result_info = move_to_target_pose(
                                        robot_interface,
                                        controller_cfg,
                                        target_delta_pose=target_delta_pose,
                                        num_steps=num_steps,
                                        num_additional_steps=num_additional_steps,
                                        interpolation_method=interpolation_method,
                                    )
                                    ep_grp.attrs["config"] = json.dumps(
                                        {
                                            "start_pose": result_info["start_pose"],
                                            "target_pose": result_info["target_pose"],
                                            "reached_pose": result_info["reached_pose"],
                                            "reached_pose_in_additional_steps": result_info[
                                                "reached_pose_in_addtional_steps"
                                            ],
                                            "exp_id": exp_id,
                                            "num_steps": num_steps,
                                            "traj_interpolation_fraction": traj_interpolation_fraction,
                                            "interpolation_method": interpolation_method,
                                            "K": {
                                                "p": Kp_translation,
                                                "d": Kp_rotation,
                                            },
                                            "residual_mass_vec": residual_mass_vec,
                                        }
                                    )
                                    ep_grp.create_dataset(
                                        "state_history",
                                        data=result_info["state_history"],
                                    )
                                    ep_grp.create_dataset(
                                        "action_history",
                                        data=result_info["action_history"],
                                    )
                                # break
                            # To be removed later
            #                 break
            #             break
            #         break
            #     break
            # break

    robot_interface.close()


if __name__ == "__main__":
    main()
