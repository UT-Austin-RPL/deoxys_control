# This is an exmaple to run cartesian velocity control with Deoxys. This controller is useful if your research is mainly about TransporterNet like tasks.
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
from deoxys.utils.config_utils import get_default_controller_config

from deoxys.experimental.motion_utils import reset_joints_to

logger = get_deoxys_example_logger()

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")

    return parser.parse_args()

def main():

    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )

    # Go back to initial joint positions
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

    time.sleep(1)

    # Cartesian velocity control

    controller_type = "CARTESIAN_VELOCITY"
    controller_cfg = get_default_controller_config(controller_type=controller_type)


    time_max = 4
    time_sequence = np.linspace(0, time_max, 100)

    v_max = 0.1
    angle = np.pi / 4.0
    for i in range(100):
        vx = 0.05 * (1 - np.abs(np.cos(np.pi * i / 100)))
        action = [vx, 0.0, vx, vx, vx * 2, vx * 4] + [-1]
        print(action[0])
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )


if __name__ == "__main__":
    main()
