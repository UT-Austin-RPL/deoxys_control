import argparse
import os
import time

import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.config_utils import robot_config_parse_args
from deoxys.utils.log_utils import get_deoxys_example_logger
from deoxys.utils.ik_utils import IKWrapper


logger = get_deoxys_example_logger()

def execute_ik_result(robot_interface, controller_type, controller_cfg, joint_traj):
    valid_input = False
    while not valid_input:
        try:
            execute = input(f"Excute or not? (enter 0 - No or 1 - Yes)")
            execute = bool(int(execute))
            valid_input = True
        except ValueError:
            print("Please input 1 or 0!")
            continue

    if execute:
        for joint in joint_traj:
            # This example assumes the gripper is open
            action = joint.tolist() + [-1.0]
            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )
    robot_interface.close()
    return bool(execute)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion-file", type=str)
    robot_config_parse_args(parser)
    return parser.parse_args()

def main():

    args = parse_args()

    # Franka Interface
    robot_interface = FrankaInterface(os.path.join(config_root, args.interface_cfg))

    controller_cfg = YamlConfig(os.path.join(config_root, "joint-impedance-controller.yml")).as_easydict()
    controller_type = "JOINT_IMPEDANCE"

    while robot_interface.state_buffer_size == 0:
        logger.warning("Robot state not received")
        time.sleep(0.5)

    last_q = np.array(robot_interface.last_q)

    last_eef_pos, last_eef_mat = robot_interface.last_eef_rot_and_pos

    ik_wrapper = IKWrapper()

    logger.info("Starting IK of goint to absolute position ...")
    target_world_position = np.array([0.4, 0.2, 0.05])
    # inverse kinematics will compute the trajectory based on the current joint configuration
    joint_traj, debug_info = ik_wrapper.ik_trajectory_to_target_position(target_world_position, last_q.tolist(), num_points=100)

    logger.info("Visualizing IK results ...")
    joint_traj = ik_wrapper.interpolate_dense_traj(joint_traj)
    ik_wrapper.simulate_joint_sequence(joint_traj)

    execute_ik_result(robot_interface, controller_type, controller_cfg, joint_traj)

    logger.info("Starting IK of moving delta position ...")
    delta_position = np.array([0.15, 0., 0.0])
    # inverse kinematics will compute the trajectory based on the current joint configuration
    joint_traj, debug_info = ik_wrapper.ik_trajectory_delta_position(delta_position, last_q.tolist(), num_points=100)

    logger.info("Visualizing IK results ...")
    joint_traj = ik_wrapper.interpolate_dense_traj(joint_traj)
    ik_wrapper.simulate_joint_sequence(joint_traj)

    execute_ik_result(robot_interface, controller_type, controller_cfg, joint_traj)
    
    
if __name__ == "__main__":
    main()
