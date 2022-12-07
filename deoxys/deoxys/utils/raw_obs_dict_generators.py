"""A compliation of multiple state observation generator."""

import numpy as np
import rpl_vision_utils.utils.img_utils as ImgUtils


class BaseRawObsDictGenerator:
    def __init__(self, *args, **kwargs):
        self.last_obs_dict = None

    def get_raw_obs_dict(self, state_info):
        """
        Args:
           state_info (dict): A dictionary of robot state + images
        """
        obs_dict = {}
        raise NotImplementedError

    def load(self):
        raise NotImplementedError


class RobotStateRawObsDictGenerator(BaseRawObsDictGenerator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def load(self, obs_dict, key, value, check_valid=True):
        """
        This is to check if the data is correct or not. Sometimes the data will be all zero depending on the networking conditions.
        """
        if (
            (
                np.sum(np.abs(value)) == 0.0
                and key in ["ee_states", "joint_states", "gripper_states"]
            )
            and check_valid
            and self.last_obs_dict is not None
        ):
            value = self.last_obs_dict[key]
        obs_dict[key] = value

    def get_raw_obs_dict(self, state_info):
        last_state = state_info["last_state"]
        last_gripper_state = state_info["last_gripper_state"]
        obs_dict = {}

        ee_states = np.array(last_state.O_T_EE)
        joint_states = np.array(last_state.q)
        gripper_states = np.array([last_gripper_state.width])

        self.load(obs_dict, "ee_states", ee_states)
        self.load(obs_dict, "joint_states", joint_states)
        # Gripper widh will probably become zero
        self.load(obs_dict, "gripper_states", gripper_states, check_valid=False)

        for state in ["ee_states", "joint_states", "gripper_states"]:
            if (
                np.sum(np.abs(obs_dict[state])) <= 1e-6
                and self.last_obs_dict is not None
            ):
                print(f"{state} missing!!!!")
                obs_dict[state] = self.last_obs_dict[state]
        self.last_obs_dict = obs_dict
        return obs_dict


class ImageRawObsDictGenerator(BaseRawObsDictGenerator):
    def __init__(self, resize_img_kwargs=None, *args, **kwargs):
        if resize_img_kwargs is None:
            resize_img_kwargs = {"k4a": {}, "rs": {}}

            resize_img_kwargs["k4a"]["img_w"] = 224
            resize_img_kwargs["k4a"]["img_h"] = 224
            resize_img_kwargs["k4a"]["fx"] = 0.35  # 0.4
            resize_img_kwargs["k4a"]["fy"] = 0.35  # 0.4

            resize_img_kwargs["rs"]["img_w"] = 224
            resize_img_kwargs["rs"]["img_h"] = 224
            resize_img_kwargs["rs"]["fx"] = 0.4
            resize_img_kwargs["rs"]["fy"] = 0.6
        self.resize_img_kwargs = resize_img_kwargs
        super().__init__(*args, **kwargs)

    def load(self, obs_dict, key, value, check_valid=True):
        """
        This is to check if the data is correct or not. Sometimes the data will be all zero depending on the networking conditions.
        """
        if (
            (
                np.sum(np.abs(value)) == 0.0
                and key in ["ee_states", "joint_states", "gripper_states"]
            )
            and check_valid
            and self.last_obs_dict is not None
        ):
            value = self.last_obs_dict[key]
        obs_dict[key] = value

    def get_raw_obs_dict(self, state_info):
        last_state = state_info["last_state"]
        last_gripper_state = state_info["last_gripper_state"]
        camera_interfaces = state_info["camera_interfaces"]

        obs_dict = {}

        ee_states = np.array(last_state.O_T_EE)
        joint_states = np.array(last_state.q)
        gripper_states = np.array([last_gripper_state.width])

        self.load(obs_dict, "ee_states", ee_states)
        self.load(obs_dict, "joint_states", joint_states)
        # Gripper widh will probably become zero
        self.load(obs_dict, "gripper_states", gripper_states, check_valid=False)

        for state in ["ee_states", "joint_states", "gripper_states"]:
            if (
                np.sum(np.abs(obs_dict[state])) <= 1e-6
                and self.last_obs_dict is not None
            ):
                print(f"{state} missing!!!!")
                obs_dict[state] = self.last_obs_dict[state]

        for camera_id in camera_interfaces.keys():
            imgs = camera_interfaces[camera_id].get_img()
            img_info = camera_interfaces[camera_id].get_img_info()
            rgb_img = imgs["color"]
            camera_type = img_info["camera_type"]
            resized_color_img = ImgUtils.resize_img(
                rgb_img, camera_type=camera_type, **self.resize_img_kwargs[camera_type]
            )

            obs_dict[f"camera_{camera_id}_color"] = resized_color_img
        self.last_obs_dict = obs_dict
        return obs_dict
