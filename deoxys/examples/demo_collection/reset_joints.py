import argparse
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from deoxys import config_root
from deoxys.camera_redis_interface import CameraRedisSubInterface
from deoxys.franka_interface import FrankaInterface
from deoxys.k4a_interface import K4aInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="alice.yml")
    parser.add_argument("--controller-cfg", type=str, default="osc-controller.yml")
    parser.add_argument(
        "--folder", type=Path, default="data_collection_example/example_data"
    )

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()

    # demo_file = h5py.File(demo_file_name, "w")
    controller_type = "JOINT_POSITION"

    # reset_joint_positions = [0.09017809387254755, -0.9824203501652151, 0.030509718397568178, -2.694229634937343, 0.057700675144720104, 1.860298714876101, 0.8713759453244422]
    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]
    # action = [0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161] + [0.]
    action = reset_joint_positions + [-1]
    while True:
        if len(robot_interface._state_buffer) > 0:
            print(robot_interface._state_buffer[-1].q)
            print(robot_interface._state_buffer[-1].q_d)
            print("-----------------------")
            # print(np.round(np.array(robot_interface._state_buffer[-1].qq) - np.array(reset_joint_positions), 5))
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

    robot_interface.close()


if __name__ == "__main__":
    main()
