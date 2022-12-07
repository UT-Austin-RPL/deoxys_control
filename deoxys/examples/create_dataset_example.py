"""
Create real robot dataset.

In this example, we create a dataset that consists of agentview images, eye-in-hand images, joint states, gripper states, and end effector states.
"""
import argparse
import os
from pathlib import Path

import cv2
import h5py
import numpy as np
from rpl_vision_utils.utils import img_utils as ImgUtils


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="config/alice.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="config/osc-controller.yml"
    )
    parser.add_argument(
        "--folder", type=Path, default="data_collection_example/example_data"
    )
    parser.add_argument("--img-h", type=int, default=224)
    parser.add_argument("--img-w", type=int, default=224)

    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    camera_ids = [0, 1]
    folder = str(args.folder)

    demo_file_name = f"{folder}/demo.hdf5"
    demo_file = h5py.File(demo_file_name, "w")

    os.makedirs(f"{folder}/data", exist_ok=True)

    grp = demo_file.create_group("data")

    # Initialize variables for saving data
    image_color_data_list = {}
    image_depth_data_list = {}

    joint_states_list = []
    ee_states_list = []
    gripper_states_list = []

    image_names_data = {}

    grp.attrs["camera_ids"] = camera_ids

    camera_name_conversion_dict = {0: "agentview_rgb", 1: "eye_in_hand_rgb"}

    if args.img_w == 224:
        fx_fy_dict = {0: {"fx": 0.35, "fy": 0.35}, 1: {"fx": 0.4, "fy": 0.6}}
    elif args.img_w == 128:
        fx_fy_dict = {0: {"fx": 0.2, "fy": 0.2}, 1: {"fx": 0.2, "fy": 0.3}}
    elif args.img_w == 84:
        fx_fy_dict = {0: {"fx": 0.13, "fy": 0.13}, 1: {"fx": 0.15, "fy": 0.225}}

    for camera_id in camera_ids:
        image_color_data_list[camera_id] = []
        image_depth_data_list[camera_id] = []

    num_demos = 0
    total_len = 0

    screenshots = {}
    for (run_idx, path) in enumerate(Path(folder).glob("run*")):
        print(run_idx)

        screenshots[run_idx] = None
        num_demos += 1
        joint_states = []
        ee_states = []
        gripper_states = []
        actions = []

        action_data = np.load(f"{path}/testing_demo_action.npz", allow_pickle=True)[
            "data"
        ]
        ee_states_data = np.load(
            f"{path}/testing_demo_ee_states.npz", allow_pickle=True
        )["data"]
        joint_states_data = np.load(
            f"{path}/testing_demo_joint_states.npz", allow_pickle=True
        )["data"]
        gripper_states_data = np.load(
            f"{path}/testing_demo_gripper_states.npz", allow_pickle=True
        )["data"]
        len_data = len(action_data)

        if len_data == 0:
            print(f"Data incorrect: {run_idx}")
            continue

        ep_grp = grp.create_group(f"demo_{run_idx}")

        camera_data = {}
        for camera_id in camera_ids:
            camera_data[camera_id] = np.load(
                f"{path}/testing_demo_camera_{camera_id}.npz", allow_pickle=True
            )["data"]

        assert len(ee_states_data) == len(
            action_data
        ), f"ee state has : {len(ee_states_data)} data, but action has {len(action_data)}"

        image_color_data = {}
        image_depth_data = {}
        image_color_names_data = {}
        image_depth_names_data = {}

        for camera_id in camera_ids:
            image_color_data[camera_id] = []
            image_depth_data[camera_id] = []
            image_color_names_data[camera_id] = []
            image_depth_names_data[camera_id] = []

        img_folder = f"{folder}/data/ep_{run_idx}/"
        os.makedirs(f"{img_folder}", exist_ok=True)

        print("Length of data", len_data)

        for i in range(len_data):
            # Rename image data and save to new folder
            # print(i, action_data[i], camera_data[0][i])
            for camera_id in camera_ids:
                image_name = f"{img_folder}/camera_{camera_id}_img_{i}"

                if "color" in camera_data[camera_id][i]:
                    color_image_name = camera_data[camera_id][i]["color"]
                    camera_type = camera_data[camera_id][i]["camera_type"]
                    img = cv2.imread(color_image_name)
                    # resized_img = cv2.resize(img, (0, 0), fx=0.2,
                    # fy=0.2)

                    if i == 0 and camera_type == "k4a":
                        screenshots[run_idx] = img
                    try:
                        resized_img = ImgUtils.resize_img(
                            img,
                            camera_type=camera_type,
                            img_w=args.img_w,
                            img_h=args.img_h,
                            fx=fx_fy_dict[camera_id]["fx"],
                            fy=fx_fy_dict[camera_id]["fy"],
                        )
                    except:
                        import pdb

                        pdb.set_trace()
                    new_image_name = f"{image_name}_color.jpg"
                    # cv2.imwrite(new_image_name, cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB))
                    image_color_data[camera_id].append(resized_img)
                    # print(resized_img.shape)
                    # image_color_names_data[camera_id].append(new_image_name)

                if "depth" in camera_data[camera_id][i] and False:
                    depth_image_name = camera_data[camera_id][i]["depth"]
                    img = cv2.imread(depth_image_name)
                    # resized_img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
                    resized_img = ImgUtils.resize_img(
                        img,
                        camera_type=camera_type,
                        img_w=args.img_w,
                        img_h=args.img_h,
                        fx=fx_fy_dict[camera_id]["fx"],
                        fy=fx_fy_dict[camera_id]["fy"],
                    )
                    new_image_name = f"{image_name}_depth.jpg"
                    cv2.imwrite(new_image_name, resized_img)
                    image_depth_data[camera_id].append(resized_img)
                    image_depth_names_data[camera_id].append(new_image_name)

            ee_states.append(ee_states_data[i])
            joint_states.append(joint_states_data[i])
            gripper_states.append([gripper_states_data[i]])
            actions.append(action_data[i])

        assert len(actions) == len(ee_states)

        assert len(image_color_data[0]) == len(actions)

        joint_states_list.append(np.stack(joint_states, axis=0))
        ee_states_list.append(np.stack(ee_states, axis=0))
        gripper_states_list.append(np.stack(gripper_states, axis=0))

        obs_grp = ep_grp.create_group("obs")
        for camera_id in camera_ids:
            obs_grp.create_dataset(
                camera_name_conversion_dict[camera_id],
                data=np.stack(image_color_data[camera_id], axis=0),
            )

        obs_grp.create_dataset("joint_states", data=np.stack(joint_states))
        obs_grp.create_dataset("ee_states", data=np.stack(ee_states))
        obs_grp.create_dataset("gripper_states", data=np.stack(gripper_states))

        ep_grp.create_dataset("actions", data=np.stack(actions))
        ep_grp.attrs["num_samples"] = len_data
        total_len += len(actions)

        os.makedirs(f"{folder}/screenshots", exist_ok=True)
        cv2.imwrite(f"{folder}/screenshots/{run_idx}.jpg", screenshots[run_idx])

    grp.attrs["num_demos"] = num_demos
    grp.attrs["total"] = total_len
    print("Total length: ", total_len)

    demo_file.close()


if __name__ == "__main__":
    main()
