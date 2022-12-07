"""
Test script for joint impedance controller - just try to reach a nearby joint position by following
an interpolated path.
"""
import argparse
import pickle
import threading
import time
from operator import truediv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-impedance-controller.yml"
    )
    parser.add_argument(
        "--folder", type=Path, default="data_collection_example/example_data"
    )

    args = parser.parse_args()
    return args


def reset(robot_interface, controller_cfg):
    """
    Reset robot to initial joint configuration.
    """
    controller_type = "JOINT_POSITION"

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
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
    logger.info("Reset finished")
    # wait for at least 1 second before returning to avoid joint impedance issue
    time.sleep(3)


def interpolate_joint_positions(start_jpos, end_jpos, num_steps=50):
    """
    Helper function to interpolate from a start joint pos to an end joint pos.
    """

    # include starting pose
    num_steps += 1

    start = np.array(start_jpos).reshape(-1)
    end = np.array(end_jpos).reshape(-1)

    # step_size = (end - start) / float(num_steps)
    # grid = np.arange(num_steps).astype(np.float64)
    # steps = np.array([start + grid[i] * step_size for i in range(num_steps)])

    # # add endpoint
    # steps = np.concatenate([steps, end[None]], axis=0)

    steps = []
    for i in range(0, num_steps + 1):
        t = float(i) * (1 / num_steps)
        transformed_step_size = 10.0 * (t**3) - 15 * (t**4) + 6 * (t**5)
        steps.append(start + transformed_step_size * (end - start))
    steps = np.array(steps)
    return steps


def move_to(robot_interface, controller_cfg, num_steps, num_additional_steps):
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
    current_q = robot_interface.last_q
    logger.info(f"Initial: {current_q}")

    target_q = np.copy(current_q)
    target_q[-1] += 0.8
    target_q[-2] += 0.8
    target_q[-3] += 0.5

    # interpolate to get joint space path
    jpos_steps = interpolate_joint_positions(
        start_jpos=current_q,
        end_jpos=target_q,
        num_steps=num_steps,
    )

    logger.info(f"Current joints: {np.round(current_q, 3)}")
    logger.info(f"Target joints: {np.round(target_q, 3)}")

    # try to follow path
    controller_type = "JOINT_IMPEDANCE"

    action_history = []
    state_history = []
    prev_action = list(current_q)
    for i, jpos_t in enumerate(jpos_steps):
        action = list(jpos_t) + [-1.0]
        logger.debug("step {}, action {}".format(i, np.round(action, 2)))
        # print("step {}, action {}".format(i, np.round(action, 2)))
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        state_history.append(np.array(robot_interface._state_buffer[-1].q))
        action_history.append(prev_action)
        prev_action = np.array(action)[:-1]

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
    import matplotlib.pyplot as plt

    plt.plot(action_history, "b.")
    plt.plot(state_history, "r-")
    plt.show()

    # print final error
    current_q = np.array(robot_interface._state_buffer[-1].q)
    logger.info("absolute error: {}".format(np.abs(target_q - current_q)))


def main():
    args = parse_args()

    # number of interpolation steps
    num_steps = 80

    # number of additional steps to freeze final target and wait
    num_additional_steps = 40

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()

    # reset arm to known configuration with joint position controller
    reset(robot_interface, controller_cfg)

    # use joint impedance controller to try and move to a new position using interpolated path
    move_to(
        robot_interface,
        controller_cfg,
        num_steps=num_steps,
        num_additional_steps=num_additional_steps,
    )

    robot_interface.close()


if __name__ == "__main__":
    main()
