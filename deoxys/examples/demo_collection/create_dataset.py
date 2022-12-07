import argparse
import os
from pathlib import Path

import cv2
import h5py
import numpy as np
from utils import resize_img


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="alice.yml")
    parser.add_argument("--controller-cfg", type=str, default="osc-controller.yml")
    parser.add_argument("--folder", type=Path, default="example_data")

    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    camera_ids = [0, 1]
    folder = str(args.folder)

    demo_file_name = f"./{folder}/demo.hdf5"
    demo_file = h5py.File(demo_file_name, "w")

    os.makedirs(f"./{folder}/data", exist_ok=True)

    grp = demo_file.create_group("data")

    # Initialize variables for saving data
    image_color_data_list = {}
    image_depth_data_list = {}

    proprio_joints_list = []
    proprio_ee_list = []
    proprio_gripper_state_list = []

    image_names_data = {}

    grp.attrs["attributes"] = ["joints", "ee", "gripper_state", "actions"]
    grp.attrs["camera_ids"] = camera_ids

    for camera_id in camera_ids:
        image_color_data_list[camera_id] = []
        image_depth_data_list[camera_id] = []

    num_eps = 0
    total_len = 0

    for (run_idx, path) in enumerate(Path(folder).glob("run*")):
        print(run_idx)

        num_eps += 1
        proprio_joints = []
        proprio_ee = []
        proprio_gripper_state = []
        actions = []

        action_data = np.load(f"{path}/testing_demo_action.npz", allow_pickle=True)[
            "data"
        ]
        proprio_ee_data = np.load(
            f"{path}/testing_demo_proprio_ee.npz", allow_pickle=True
        )["data"]
        proprio_joints_data = np.load(
            f"{path}/testing_demo_proprio_joints.npz", allow_pickle=True
        )["data"]
        proprio_gripper_state_data = np.load(
            f"{path}/testing_demo_proprio_gripper_state.npz", allow_pickle=True
        )["data"]
        len_data = len(action_data)

        if len_data == 0:
            print(f"Data incorrect: {run_idx}")
            continue

        ep_grp = grp.create_group(f"ep_{run_idx}")

        camera_data = {}
        for camera_id in camera_ids:
            camera_data[camera_id] = np.load(
                f"{path}/testing_demo_camera_{camera_id}.npz", allow_pickle=True
            )["data"]

        assert len(proprio_ee_data) == len(action_data)

        image_color_data = {}
        image_depth_data = {}
        image_color_names_data = {}
        image_depth_names_data = {}

        for camera_id in camera_ids:
            image_color_data[camera_id] = []
            image_depth_data[camera_id] = []
            image_color_names_data[camera_id] = []
            image_depth_names_data[camera_id] = []

        img_folder = f"./{folder}/data/ep_{run_idx}/"
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

                    try:
                        resized_img = resize_img(img, camera_type=camera_type)
                    except:
                        import pdb

                        pdb.set_trace()
                    new_image_name = f"{image_name}_color.jpg"
                    cv2.imwrite(
                        new_image_name, cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB)
                    )
                    image_color_data[camera_id].append(resized_img.transpose(2, 0, 1))
                    image_color_names_data[camera_id].append(new_image_name)

                if "depth" in camera_data[camera_id][i]:
                    depth_image_name = camera_data[camera_id][i]["depth"]
                    img = cv2.imread(depth_image_name)
                    # resized_img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
                    resized_img = resize_img(img, camera_type=camera_type)
                    new_image_name = f"{image_name}_depth.jpg"
                    cv2.imwrite(new_image_name, resized_img)
                    image_depth_data[camera_id].append(resized_img.transpose(2, 0, 1))
                    image_depth_names_data[camera_id].append(new_image_name)

            proprio_ee.append(proprio_ee_data[i])
            proprio_joints.append(proprio_joints_data[i])
            proprio_gripper_state.append([proprio_gripper_state_data[i]])
            actions.append(action_data[i])

        assert len(actions) == len(proprio_ee)

        assert len(image_color_data[0]) == len(actions)

        proprio_joints_list.append(np.stack(proprio_joints, axis=0))
        proprio_ee_list.append(np.stack(proprio_ee, axis=0))
        proprio_gripper_state_list.append(np.stack(proprio_gripper_state, axis=0))

        for camera_id in camera_ids:
            ep_data_grp = ep_grp.create_dataset(
                f"camera_{camera_id}_color", data=image_color_names_data[camera_id]
            )
            # ep_data_grp = ep_grp.create_dataset(f"camera_{camera_id}_depth", data=image_depth_names_data[camera_id])
            image_color_data_list[camera_id].append(
                np.stack(image_color_data[camera_id], axis=0)
            )
            # image_depth_data_list[camera_id].append(np.stack(image_depth_data[camera_id], axis=0))

        ep_data_grp = ep_grp.create_dataset(
            "proprio_joints", data=np.stack(proprio_joints)
        )
        ep_data_grp = ep_grp.create_dataset("proprio_ee", data=np.stack(proprio_ee))
        ep_data_grp = ep_grp.create_dataset(
            "proprio_gripper_state", data=np.stack(proprio_gripper_state)
        )

        ep_data_grp = ep_grp.create_dataset("actions", data=np.stack(actions))
        total_len += len(actions)
    grp.create_dataset("proprio_ee", data=np.vstack(proprio_ee_list))
    grp.create_dataset("proprio_joints", data=np.vstack(proprio_joints_list))
    grp.create_dataset(
        "proprio_gripper_state", data=np.vstack(proprio_gripper_state_list)
    )

    # Put images into demo.hdf5
    for camera_id in camera_ids:
        grp.create_dataset(
            f"camera_{camera_id}_images_color",
            data=np.vstack(image_color_data_list[camera_id]),
        )
        # grp.create_dataset(f"camera_{camera_id}_images_depth", data=np.vstack(image_depth_data_list[camera_id]))

    grp.attrs["color_img_h"] = image_color_data[camera_id][0].shape[1]
    grp.attrs["color_img_w"] = image_color_data[camera_id][0].shape[2]

    # grp.attrs["depth_img_h"] = image_depth_data[camera_id][0].shape[1]
    # grp.attrs["depth_img_w"] = image_depth_data[camera_id][0].shape[2]

    grp.attrs["num_eps"] = num_eps
    grp.attrs["len"] = total_len
    print("Total length: ", total_len)

    print(grp.attrs["color_img_h"], grp.attrs["color_img_w"])

    demo_file.close()


if __name__ == "__main__":
    main()
