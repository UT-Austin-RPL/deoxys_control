"""This is a script for kinesthetic teaching."""
import argparse
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import simplejson as json
from experimental import follow_joint_traj, reset_joints_to

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse


def demonstration():
    device = SpaceMouse(vendor_id=9583, product_id=50734)
    device.start_control()

    robot_interface = FrankaInterface(
        config_root + "/charmander.yml", use_visualizer=False
    )
    controller_cfg = YamlConfig(
        config_root + "/compliant-joint-impedance-controller.yml"
    ).as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    recorded_joints = []

    time.sleep(1.0)
    while True:
        spacemouse_action, grasp = input2action(
            device=device,
            controller_type="OSC_POSE",
        )

        if spacemouse_action is None:
            break

        if robot_interface.check_nonzero_configuration():
            current_joint_pos = np.array(robot_interface.last_q)
            recorded_joints.append(current_joint_pos.tolist())
        else:
            continue
        action = current_joint_pos.tolist() + [-1]
        # print(np.array(robot_interface._state_buffer[-1].O_T_EE).reshape(4, 4).transpose()[:3, 3:])
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )

    while True:
        try:
            save = int(input("save or not? (1 - Yes, 0 - No)"))
        except ValueError:
            print("Please input 1 or 0!")
            continue
        break

    if save:
        config_folder = "experimental_results"
        os.makedirs(config_folder, exist_ok=True)
        file_name = "example.json"
        joint_info_json_filename = f"{config_folder}/{file_name}"

        with open(joint_info_json_filename, "w") as f:
            json.dump({"joints": recorded_joints}, f, indent=4)

    robot_interface.close()


def replay_demonstration():
    robot_interface = FrankaInterface(
        config_root + "/charmander.yml", use_visualizer=False
    )
    # controller_cfg = YamlConfig(config_root + "/joint-impedance-controller.yml").as_easydict()
    # controller_type = "JOINT_IMPEDANCE"

    config_folder = "experimental_results"
    os.makedirs(config_folder, exist_ok=True)
    file_name = "example.json"
    joint_info_json_filename = f"{config_folder}/{file_name}"

    with open(joint_info_json_filename, "r") as f:
        recorded_joints = json.load(f)["joints"]
    reset_joints_to(robot_interface, recorded_joints[0])
    joint_pos_history, action_history = follow_joint_traj(
        robot_interface, recorded_joints
    )
    robot_interface.close()
    plt.plot(np.array(joint_pos_history), "ro")
    plt.plot(np.array(action_history), "b")
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--demonstration", action="store_true")

    args = parser.parse_args()

    if args.demonstration:
        demonstration()
    else:
        replay_demonstration()


if __name__ == "__main__":
    main()
