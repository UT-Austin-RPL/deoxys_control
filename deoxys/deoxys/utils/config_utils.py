import argparse
import logging
import os
import pprint
from asyncore import file_dispatcher
from pathlib import Path
from typing import Type

import numpy as np
from easydict import EasyDict

from deoxys import config_root
from deoxys.utils.yaml_config import YamlConfig

logger = logging.getLogger(__name__)


def add_robot_config_arguments(parser):
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--folder", type=Path, default="/tmp/deoxys_demo_data")


def add_controller_config_arguments(parser):
    parser.add_argument("--controller-type", type=str, default="OSC_POSE")
    parser.add_argument(
        "--controller-cfg", type=str, default="osc-position-controller.yml"
    )


def robot_config_parse_args(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
        add_robot_config_arguments(parser)
        add_controller_config_arguments(parser)
        args = parser.parse_args()
        return args
    else:
        add_robot_config_arguments(parser)
        add_controller_config_arguments(parser)

# Controller configs


def print_controller_config_from_file(config_file_name):
    config = YamlConfig(config_file_name).as_easydict()
    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(config)


def get_available_controller_configs(config_folder: str = None, verbose: bool = False):
    """_summary_

    Args:
        config_folder (str, optional): the folder to look for controller configs. Defaults to None.
        verbose (bool, optional): boolean flag to determine whether to read the found controller config file and print out its information. Defaults to False.
    """
    if config_folder is None:
        config_folder = config_root
    config_dict = {}
    for config_file_name in Path(config_folder).glob("*controller.yml"):
        if verbose:
            print("Found file: ", config_file_name)
            print_controller_config_from_file(config_file_name)
        config = YamlConfig(config_file_name).as_easydict()
        config_dict[config["controller_type"]] = config
    return config_dict


def get_default_controller_config(controller_type: str) -> EasyDict:
    """Get the default dictionary of controller configuration that corresponds to the given controller type.

    Args:
        controller_type (str): Controller type name.

    Returns:
        Type(EasyDict): An easy dictionary of controller configuration
    """
    print(controller_type)
    if controller_type == "OSC_POSE":
        controller_cfg = YamlConfig(
            os.path.join(config_root, "osc-pose-controller.yml")
        ).as_easydict()
        controller_cfg = verify_controller_config(controller_cfg)
    elif controller_type == "OSC_POSITION":
        controller_cfg = YamlConfig(
            os.path.join(config_root, "osc-position-controller.yml")
        ).as_easydict()
        controller_cfg = verify_controller_config(controller_cfg)
    elif controller_type == "OSC_YAW":
        controller_cfg = YamlConfig(
            os.path.join(config_root, "osc-yaw-controller.yml")
        ).as_easydict()
        controller_cfg = verify_controller_config(controller_cfg)
    elif controller_type == "JOINT_IMPEDANCE":
        controller_cfg = YamlConfig(
            os.path.join(config_root, "joint-impedance-controller.yml")
        ).as_easydict()
        controller_cfg = verify_controller_config(controller_cfg)
        logging.warning(
            "Default joint impedance config is chosen to be rigid. Specify your own config if you want to have compliant mode."
        )
    elif controller_type == "JOINT_POSITION":
        controller_cfg = YamlConfig(
            os.path.join(config_root, "joint-position-controller.yml")
        ).as_easydict()
        controller_cfg = verify_controller_config(controller_cfg)
    elif controller_type == "CARTESIAN_VELOCITY":
        controller_cfg = YamlConfig(
            os.path.join(config_root, "cartesian-velocity-controller.yml")
        ).as_easydict()
    return controller_cfg


def check_attr(cfg, attr_key):
    return hasattr(cfg, attr_key) and cfg[attr_key] is not None


def verify_controller_config(controller_cfg: dict, use_default=True):
    """Verify if the controller configuration has defined all necessary configs.

    Args:
        controller_cfg (dict): The controller config
        use_default (bool, optional): Use default values for fields if they are None. Defaults to True.
        warning (bool, optional): Print warning when fields in config are not specified. Defaults to False.

    Returns:
        dict : a verified and updated controller config
    """
    assert (
        "controller_type" in controller_cfg
    ), "The controller config must specify the controller type"

    field_missing = False

    if controller_cfg["controller_type"] in ["OSC_POSE", "OSC_YAW", "OSC_POSITION"]:
        # Control gains
        if not check_attr(controller_cfg, "Kp"):
            controller_cfg["Kp"] = {"translation": [150.0] * 3, "rotation": [250.0] * 3}
            logger.warning("field Kp is not specified!!!")
            field_missing = True
        else:
            if not check_attr(controller_cfg["Kp"], "translation"):
                controller_cfg["Kp"]["translation"] = [150.0] * 3
                logger.warning("translation stiffness is not specified!!!")
                field_missing = True
            elif type(controller_cfg["Kp"]["translation"]) is float:
                controller_cfg["Kp"]["translation"] = [
                    controller_cfg["Kp"]["translation"]
                ] * 3
            if not check_attr(controller_cfg["Kp"], "rotation"):
                controller_cfg["Kp"]["rotation"] = [250.0] * 3
                logger.warning("rotational stiffness is not specified!!!")
                field_missing = True
            elif type(controller_cfg["Kp"]["rotation"]) is float:
                controller_cfg["Kp"]["rotation"] = [
                    controller_cfg["Kp"]["rotation"]
                ] * 3
        assert len(controller_cfg["Kp"]["translation"]) == 3
        assert len(controller_cfg["Kp"]["rotation"]) == 3
        # Trajectory interpolation
        if not check_attr(controller_cfg, "traj_interpolator_cfg"):
            controller_cfg["traj_interpolator_cfg"] = {
                "traj_interpolater_type": "LINEAR_POSE",
                "time_fraction": 0.3,
            }
            logger.warning("field traj_interpolator_cfg not specified!!!")
            field_missing = True
        # Residual mass vector
        if not check_attr(controller_cfg, "residual_mass_vec"):
            controller_cfg["residual_mass_vec"] = [0.0] * 4 + [0.1] * 3
            logger.warning("field residual_mass_vec not specified!!!")
            field_missing = True
        else:
            assert len(controller_cfg.residual_mass_vec) == 7
        # Action scale for OSC controllers
        if not check_attr(controller_cfg, "action_scale"):
            controller_cfg["action_scale"] = {"translation": 0.05, "rotation": 1.0}
            logger.warning("field action_scale not specified!!!")
        else:
            if not check_attr(controller_cfg["action_scale"], "translation"):
                controller_cfg["action_scale"]["translation"] = 0.05
                logger.warning("field translation in action_scale not specified!!!")
                field_missing = True
            if not check_attr(controller_cfg["action_scale"], "rotation"):
                controller_cfg["action_scale"]["rotation"] = 1.0
                logger.warning("field rotation in action_scale not specified!!!")
                field_missing = True

    elif controller_cfg["controller_type"] == "JOINT_IMPEDANCE":
        if not check_attr(controller_cfg, "traj_interpolator_cfg"):
            controller_cfg["traj_interpolator_cfg"] = {
                "traj_interpolater_type": "LINEAR_JOINT_POSITION",
                "time_fraction": 0.3,
            }
            logger.warning("field traj_interpolator_cfg not specified!!!")
            field_missing = True
        if not check_attr(controller_cfg, "joint_kp"):
            controller_cfg.joint_kp = [5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0]
            logger.warning("field joint_kp not manually specified!!!")
            field_missing = True
        if not check_attr(controller_cfg, "joint_kd"):
            controller_cfg.joint_kd = 2 * np.sqrt(controller_cfg.joint_kp).tolist()
            logger.debug(
                "field joint_kd not manually specified, switch to the critical damping formula."
            )
            field_missing = True

    elif controller_cfg["controller_type"] == "JOINT_POSITION":
        if not check_attr(controller_cfg, "traj_interpolator_cfg"):
            controller_cfg["traj_interpolator_cfg"] = {
                "traj_interpolater_type": "SMOOTH_JOINT_POSITION",
                "time_fraction": 0.3,
            }
            logger.warning("field traj_interpolator_cfg not specified!!!")
            field_missing = True
    
    elif controller_cfg["controller_type"] == "CARTESIAN_VELOCITY":
        if not check_attr(controller_cfg, "traj_interpolator_cfg"):
            controller_cfg["traj_interpolator_cfg"] = {
                "traj_interpolater_type": "NULL_VELOCITY",
                "time_fraction": 0.3,
            }
            logger.warning("field traj_interpolator_cfg not specified!!!")
            field_missing = True
    
    if field_missing and not use_default:
        logger.error(
            "Some field in controller is not specified and default config option is not turned on!!!"
        )
        raise ValueError

    return controller_cfg
