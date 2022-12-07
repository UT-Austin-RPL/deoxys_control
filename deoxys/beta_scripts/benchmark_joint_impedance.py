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

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-impedance-controller.yml"
    )
    parser.add_argument(
        "--folder", type=Path, default="benchmark_results/joint_impedance"
    )

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


def interpolate_joint_positions(
    start_jpos, end_jpos, num_steps=50, interpolation_method="linear"
):
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


def move_to_target_joints(
    robot_interface,
    controller_cfg,
    target_q,
    num_steps,
    num_additional_steps,
    interpolation_method,
):
    """
    Use joint impedance controller to move to a new joint position using interpolation.
    """
    while True:
        if len(robot_interface._state_buffer) > 0:
            if np.max(np.abs(np.array(robot_interface._state_buffer[-1].q))) < 1e-3:
                print(
                    len(robot_interface._state_buffer),
                    np.array(robot_interface._state_buffer[-1].q),
                )
                continue
            else:
                break
    current_q = np.array(robot_interface._state_buffer[-1].q)
    # interpolate to get joint space path
    jpos_steps = interpolate_joint_positions(
        start_jpos=current_q,
        end_jpos=target_q,
        num_steps=num_steps,
        interpolation_method=interpolation_method,
    )

    print("current: ", current_q)
    print("target: ", target_q)
    # try to follow path
    controller_type = "JOINT_IMPEDANCE"

    action_history = []
    state_history = []
    prev_action = list(current_q)
    for i, jpos_t in enumerate(jpos_steps):
        action = list(jpos_t) + [-1.0]

        # print("step {}, action {}".format(i, np.round(action, 2)))
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        state_history.append(np.array(robot_interface._state_buffer[-1].q))
        action_history.append(prev_action)
        prev_action = np.array(action)[:-1]
    reached_q = state_history[-1]
    for i in range(num_additional_steps):
        action = list(target_q) + [-1.0]
        # print("additional step {}, action {}".format(i, np.round(action, 2)))

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        state_history.append(np.array(robot_interface._state_buffer[-1].q))
        action_history.append(prev_action)
        prev_action = np.array(action)[:-1]

    for _ in range(30):
        action_history.append(np.array(action)[:-1])
        state_history.append(np.array(robot_interface._state_buffer[-1].q))

    action_history = np.array(action_history)
    state_history = np.array(state_history)
    # print final error
    current_q = np.array(robot_interface._state_buffer[-1].q)
    print("absolute error: {}".format(np.abs(target_q - current_q)))
    return {
        "target_q": target_q,
        "reached_q": reached_q.tolist(),
        "reached_q_in_addtional_steps": current_q.tolist(),
        "state_history": state_history,
        "action_history": action_history,
    }


def main():
    args = parse_args()

    os.makedirs(args.folder, exist_ok=True)

    # We test the following configurations, ranging from high-level trajectory planning to low level control hyperparameters
    # number of interplation steps
    num_exp = 3
    num_steps_list = [40, 60, 80, 100]
    num_additional_steps = 40
    traj_interpolation_fraction_list = [0.1, 0.3, 0.5, 0.7, 0.9, 1.0]
    K_list = [
        (
            [100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0],
            [20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0],
        ),
        ([100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0], None),
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

    joint_configuration_tuples = [
        (
            reset_joint_positions,
            [
                0.07711386684790111,
                0.45387076125665143,
                -0.031040170910847997,
                -1.6380967056795717,
                -0.013967671018826193,
                2.2215563491727406,
                0.8438968810082668,
            ],
        ),  # Move forward
        (
            reset_joint_positions,
            [
                0.30698764864812084,
                0.1969125487101344,
                0.2688113091647901,
                -1.9466874116704824,
                0.00775540224679154,
                2.2377662752365812,
                1.4325957019205537,
            ],
        ),  # Move left
        (
            reset_joint_positions,
            [
                -0.27446286256733693,
                0.00728192711392777,
                -0.39757973366999044,
                -2.2105146014899537,
                0.0017035714901155895,
                2.2991181914629677,
                0.16698454174731334,
            ],
        ),  # Move right
        (
            reset_joint_positions,
            [
                0.03397621909216041,
                -0.9397979360849174,
                -0.05754924549943754,
                -2.9206017485627167,
                -0.014663805255299709,
                1.9951406412787318,
                0.8439047071006561,
            ],
        ),  # Move backward
        (
            reset_joint_positions,
            [
                0.08971790130723985,
                -0.2556630445446884,
                -0.03925560523566679,
                -1.5852414443594847,
                0.025609309734569652,
                1.3862748274737364,
                0.9327958988481097,
            ],
        ),  # Move up
        (
            reset_joint_positions,
            [
                0.08686263975867053,
                0.3567868885900756,
                -0.044078772430180456,
                -2.5205687444921123,
                -0.012724554827643763,
                2.8709642396747994,
                0.8504274967889236,
            ],
        ),  # Move down
        (
            reset_joint_positions,
            [
                -0.1746833600312815,
                0.10496730036947034,
                -0.372185451628869,
                -1.5574972174079889,
                0.9539888957761783,
                1.360130403571168,
                0.4815867085589302,
            ],
        ),  # rotate x axis + move
        (
            reset_joint_positions,
            [
                0.013130354527275116,
                0.6372912316667938,
                -0.022820039241258205,
                -1.4311305010461124,
                0.05441758368156915,
                1.6585589204496802,
                0.8427739579270945,
            ],
        ),  # rotate y axis + move
        (
            reset_joint_positions,
            [
                0.08053155546787921,
                0.02281445671032479,
                -0.01633931171709141,
                -1.8815259116839136,
                -0.010511146908832921,
                1.9331576181612105,
                2.3015269843075012,
            ],
        ),  # rotate z axis + move
        (
            reset_joint_positions,
            [
                -0.2249948008123197,
                -0.16249597452772127,
                -0.42947307634818027,
                -2.013705242353301,
                0.7909554669516845,
                2.736800716943909,
                1.0574495259258483,
            ],
        ),  # mix rotate + move 1
        (
            reset_joint_positions,
            [
                0.312281660793071,
                0.015310423421769845,
                0.40427782744901214,
                -1.7347451199114312,
                -1.092601773614446,
                2.657680732634332,
                0.007125373875313622,
            ],
        ),  # mix rotate + move 2
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
                for traj_interpolation_fraction in traj_interpolation_fraction_list:
                    for interpolation_method in ["linear", "min_jerk"]:
                        for joint_kp, joint_kd in K_list:
                            for start_q, target_q in joint_configuration_tuples:
                                count += 1
                                ep_grp = data_grp.create_group(f"{exp_id}_{count}")
                                controller_cfg.joint_kp = joint_kp
                                controller_cfg.joint_kd = joint_kd
                                # loop through each set of hyperparameter
                                # loop through each joint configuration and record data
                                reset_to_joints(
                                    robot_interface, controller_cfg, start_q
                                )
                                result_info = move_to_target_joints(
                                    robot_interface,
                                    controller_cfg,
                                    target_q=target_q,
                                    num_steps=num_steps,
                                    num_additional_steps=num_additional_steps,
                                    interpolation_method=interpolation_method,
                                )
                                ep_grp.attrs["config"] = json.dumps(
                                    {
                                        "start_q": start_q,
                                        "target_q": target_q,
                                        "reached_q": result_info["reached_q"],
                                        "reached_q_in_additional_steps": result_info[
                                            "reached_q_in_addtional_steps"
                                        ],
                                        "exp_id": exp_id,
                                        "num_steps": num_steps,
                                        "traj_interpolation_fraction": traj_interpolation_fraction,
                                        "interpolation_method": interpolation_method,
                                        "K": {"p": joint_kp, "d": joint_kd},
                                    }
                                )
                                ep_grp.create_dataset(
                                    "state_history", data=result_info["state_history"]
                                )
                                ep_grp.create_dataset(
                                    "action_history", data=result_info["action_history"]
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
