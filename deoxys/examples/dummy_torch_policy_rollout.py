"""This example is to show a dummy control policy to carry out a robot rollout."""
import argparse
import json
import os
import pprint
import sys
from pathlib import Path

import cv2
import h5py
import hydra
import numpy as np
import torch
import torchvision
import yaml
from easydict import EasyDict
from hydra.experimental import compose, initialize
from omegaconf import DictConfig, OmegaConf
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torch.utils.tensorboard import SummaryWriter
from torchvision import transforms

from deoxys import config_root
# Control import
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import RobotStateRawObsDictGenerator, YamlConfig
from deoxys.utils.config_utils import robot_config_parse_args
from deoxys.utils.input_utils import input2action
from deoxys.utils.io_devices import SpaceMouse
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


class ObsPreprocessorForEval:
    def __init__(self, cfg=None, visual_key_mapping={}, proprio_key_mapping={}):

        self.cfg = cfg
        self.visual_key_mapping = visual_key_mapping
        self.prorpio_key_mapping = proprio_key_mapping
        self.gripper_history = []

    def reset(self):
        self.gripper_history = []

    def map_proprio_name(self, name):
        if name in self.proprio_key_mapping:
            return self.proprio_key_mapping[name]
        else:
            return name

    def get_obs(self, obs):
        data = {"obs": {}}

        if self.gripper_history == []:
            for _ in range(5):
                self.gripper_history.append(
                    torch.from_numpy(obs[self.map_proprio_name("gripper_states")])
                )
        self.gripper_history.pop(0)
        self.gripper_history.append(
            torch.from_numpy(obs[self.map_proprio_name("gripper_states")])
        )
        data["obs"]["gripper_history"] = torch.cat(self.gripper_history, dim=-1).float()
        for proprio_state_key, obs_key in self.proprio_key_mapping.items():
            data["obs"][proprio_state_key] = torch.from_numpy(obs[obs_key]).float()


def main():
    logger.warn("This is a very dummy control policy!!!!!")

    args = robot_config_parse_args()
    robot_interface = FrankaInterface(os.path.join(config_root, args.interface_cfg))
    controller_cfg = YamlConfig(
        os.path.join(config_root, args.controller_cfg)
    ).as_easydict()
    controller_type = args.controller_type

    device = SpaceMouse(vendor_id=9583, product_id=50734)
    device.start_control()

    raw_obs_dict_generator = RobotStateRawObsDictGenerator()

    import time

    time.sleep(0.3)
    dummy_torch_model = torch.nn.Linear(7, 1)

    for _ in range(1000):
        action, grasp = input2action(
            device=device,
            controller_type=controller_type,
        )
        if action is None:
            break
        if len(robot_interface._state_buffer) == 0:
            continue

        # Generate observation dictionary

        # (Advanced) more processing if your model needs custom processing

        last_state = robot_interface._state_buffer[-1]
        last_gripper_state = robot_interface._gripper_state_buffer[-1]

        obs_dict = raw_obs_dict_generator.get_raw_obs_dict(
            {"last_state": last_state, "last_gripper_state": last_gripper_state}
        )

        print(obs_dict)
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
    robot_interface.close()


if __name__ == "__main__":
    main()
