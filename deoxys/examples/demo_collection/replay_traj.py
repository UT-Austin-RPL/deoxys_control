import argparse
import os
import pickle
import threading
import time
from pathlib import Path

import h5py
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
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-position-controller.yml"
    )
    parser.add_argument("--folder", type=Path, default="example_data")

    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    # In this example, we replay the first trajectory in
    # {args.folder}/demo.hdf5

    # # Initialize cameras if you want to record new images
    # camera_ids = [0, 1]
    # cr_interfaces = {}
    # for camera_id in camera_ids:
    #     cr_interface = CameraRedisSubInterface(camera_id=camera_id)
    #     cr_interface.start()
    #     cr_interfaces[camera_id] = cr_interface

    # Initialize robot interface
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()
    controller_type = "OSC_POSITION"

    robot_interface = FrankaInterface(config_root + f"/{args.interface_cfg}")

    demo_file_name = str(args.folder / "demo.hdf5")
    demo = h5py.File(demo_file_name, "r")

    episode = demo["data/ep_0"]

    actions = episode["actions"][()]

    for action in actions:
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

    robot_interface.close()


if __name__ == "__main__":
    main()
