"""This is a script for kinesthetic teaching."""
import argparse
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import simplejson as json

from deoxys import config_root
from deoxys.experimental.motion_utils import position_only_gripper_move_by
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def main():
    logger.warning("This is a beta script")
    robot_interface = FrankaInterface(
        config_root + "/charmander.yml", use_visualizer=False
    )
    logger.debug("Robot interface initalized")
    position_only_gripper_move_by(robot_interface, delta_pos=[0.2, 0.05, 0], grasp=False)
    logger.debug("First movement finished")
    position_only_gripper_move_by(robot_interface, delta_pos=[0.0, 0.0, -0.22], grasp=False)
    logger.debug("Second movement finished")
    position_only_gripper_move_by(robot_interface, delta_pos=[0.0, 0.0, 0.0], grasp=True)
    logger.debug("Third movement finished")
    position_only_gripper_move_by(robot_interface, delta_pos=[0.0, 0.0, 0.22], grasp=True)    
    logger.debug("Forth movement finished")
    position_only_gripper_move_by(
        robot_interface, delta_pos=[-0.2, -0.05, 0.0], grasp=True
    )
    position_only_gripper_move_by(
        robot_interface, delta_pos=[0.0, 0.00, -0.10], grasp=True
    )    
    logger.debug("Final movement finished")
    position_only_gripper_move_by(
        robot_interface, delta_pos=[0.0, 0.0, 0.0], grasp=False
    )
    logger.error("NO errors. Just need to test error message")


if __name__ == "__main__":
    main()
