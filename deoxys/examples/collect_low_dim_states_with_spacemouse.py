"""Teleoperating robot arm with a SpaceMouse to collect demonstration data"""

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
from deoxys.utils.config_utils import (add_robot_config_arguments,
                                       get_default_controller_config)
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vendor_id",
        type=int,
        default=9583,
    )
    parser.add_argument(
        "--product_id",
        type=int,
        default=50734,
    )
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-position-controller.yml"
    )
    add_robot_config_arguments(parser)
    return parser.parse_args()


def main():
    args = parse_args()

    args.folder.mkdir(parents=True, exist_ok=True)

    experiment_id = 0

    logger.info(f"Saving to {args.folder}")

    # Create a folder that saves the demonstration raw states.
    for path in args.folder.glob("run*"):
        if not path.is_dir():
            continue
        try:
            folder_id = int(str(path).split("run")[-1])
            if folder_id > experiment_id:
                experiment_id = folder_id
        except BaseException:
            pass

    experiment_id += 1
    folder = str(args.folder / f"run{experiment_id}")

    device = SpaceMouse(vendor_id=args.vendor_id, product_id=args.product_id)
    device.start_control()

    # Franka Interface
    if args.interface_cfg[0] != "/":
        config_path = os.path.join(config_root, args.interface_cfg)
    else:
        config_path = args.interface_cfg
    robot_interface = FrankaInterface(config_path)

    controller_type = args.controller_type

    controller_cfg = YamlConfig(
        os.path.join(config_root, args.controller_cfg)
    ).as_easydict()

    # If controller_cfg is different from user's specified controller
    # type, throw a warning and fall back to defautl config
    if controller_cfg.controller_type != controller_type:
        logger.warn("Config specification mismatched, using default controller config")
        controller_cfg = get_default_controller_config(controller_type)

    data = {"action": [], "ee_states": [], "joint_states": [], "gripper_states": []}
    i = 0
    start = False

    previous_state_dict = None

    time.sleep(2)

    while i < 4000:
        logger.info(i)
        i += 1
        start_time = time.time_ns()
        action, grasp = input2action(
            device=device,
            controller_type=controller_type,
        )
        if action is None:
            break

        # set unused orientation dims to 0
        if controller_type == "OSC_YAW":
            action[3:5] = 0.0
        elif controller_type == "OSC_POSITION":
            action[3:6] = 0.0
        elif controller_type == "JOINT_IMPEDANCE":
            action[:7] = robot_interface.last_q.tolist()
        logger.info(action)

        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

        if len(robot_interface._state_buffer) == 0:
            continue
        last_state = robot_interface._state_buffer[-1]
        last_gripper_q = robot_interface.last_gripper_q
        if np.linalg.norm(action[:-1]) < 1e-3 and not start:
            continue

        start = True
        # print(action.shape)
        # Record ee pose,  joints

        data["action"].append(action)

        state_dict = {
            "ee_states": np.array(last_state.O_T_EE),
            "joint_states": np.array(last_state.q),
        }

        if last_gripper_q is not None:
            state_dict.update(
                {
                    "gripper_states": last_gripper_q,
                }
            )

        if previous_state_dict is not None:
            for proprio_key in state_dict.keys():
                proprio_state = state_dict[proprio_key]
                if np.sum(np.abs(proprio_state)) <= 1e-6:
                    proprio_state = previous_state_dict[proprio_key]
                state_dict[proprio_key] = np.copy(proprio_state)
        for proprio_key in state_dict.keys():
            data[proprio_key].append(state_dict[proprio_key])

        previous_state_dict = state_dict

        end_time = time.time_ns()
        print(f"Time profile: {(end_time - start_time) / 10 ** 9}")
    os.makedirs(folder, exist_ok=True)

    with h5py.File(f"{folder}/recorded_trajecotry.hdf5", "w") as h5py_file:
        config_dict = {
            "controller_cfg": dict(controller_cfg),
            "controller_type": controller_type,
        }
        grp = h5py_file.create_group("data")
        grp.attrs["config"] = json.dumps(config_dict)

        grp.create_dataset("actions", data=np.array(data["action"]))
        grp.create_dataset("ee_states", data=np.array(data["ee_states"]))
        grp.create_dataset("joint_states", data=np.array(data["joint_states"]))
        grp.create_dataset("gripper_states", data=np.array(data["gripper_states"]))

    robot_interface.close()
    logger.info("Total length of the trajectory: {}".format(len(data["action"])))
    logger.info(f"The trajectory info is saved in {folder}/recorded_trajecotry.hdf5")
    valid_input = False
    while not valid_input:
        try:
            save = input("Save or not? (enter 0 or 1)")
            save = bool(int(save))
            valid_input = True
        except:
            pass
    if not save:
        import shutil

        shutil.rmtree(f"{folder}")


if __name__ == "__main__":
    main()
