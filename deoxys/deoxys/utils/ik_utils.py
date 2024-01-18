"""This implementation of inverse kinematics is largely based on the version from dm_control. https://github.com/google-deepmind/dm_control/blob/main/dm_control/utils/inverse_kinematics.py"""

import os
import time
import mujoco
from mujoco import viewer
import numpy as np

from tqdm import trange
from deoxys import ROOT_PATH

class IKWrapper:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(os.path.join(ROOT_PATH, "./assets/franka_emika_panda/scene.xml"))
        self.data = mujoco.MjData(self.model)

    def ik_trajectory_from_T_seq(self, T_seq, start_joint_positions, verbose=True):
        assert(len(start_joint_positions) == 7), "start_joint_positions should be a list of 7 elements"
        predicted_joints_seq = [np.array(start_joint_positions)]

        self.data.qpos[:] = start_joint_positions + [0.04] * 2
        gripper_site_id = self.model.site("grip_site").id
        jac = np.zeros((6, self.model.nv))

        mujoco.mj_step(self.model, self.data, 1)
        current_pos = np.copy(self.data.site(gripper_site_id).xpos)
        current_mat = np.copy(self.data.site(gripper_site_id).xmat).reshape(3, 3)
        mujoco.mj_jacSite(self.model, self.data, jac[:3], jac[3:], gripper_site_id)

        new_T = np.eye(4)
        new_T[:3, :3] = current_mat
        new_T[:3, 3] = current_pos

        new_T_seq = []
        for i in range(len(T_seq)):
            T = T_seq[i]
            new_T = T @ new_T
            new_T_seq.append(new_T)

        if verbose:
            print("Current eef (pos): ", current_pos)
            print("Predicted goal eef (pos): ", new_T[:3, 3])
            print("--------------------")
            print("Current eef (mat): ", current_mat)
            print("Predicted goal eef (mat): ", new_T[:3, :3])
        
        target_positions = []
        target_mats = []
        # new_T_seq = [new_T_seq[-1]]

        quat_seq = []
        for i in trange(len(new_T_seq)):
            new_quat = np.empty(4)
            target_mat = new_T_seq[i][:3, :3]
            mujoco.mju_mat2Quat(new_quat, target_mat.reshape(9, 1))
            quat_seq.append(new_quat)
            target_pos = new_T_seq[i][:3, 3]
            predicted_joints = self.inverse_kinematics(self.model, self.data, target_mat, target_pos, start_joint_positions)
            predicted_joints_seq.append(predicted_joints)
        debug_info = {
            "predicted_joints_seq": predicted_joints_seq,
            "new_T_seq": new_T_seq,
            "quat_seq": quat_seq
        }
        return predicted_joints_seq, debug_info

    def ik_trajectory_delta_position(self, delta_pos, start_joint_positions, num_points=100, verbose=True):
        assert(len(start_joint_positions) == 7), "start_joint_positions should be a list of 7 elements"
        predicted_joints_seq = [np.array(start_joint_positions)]

        self.data.qpos[:] = start_joint_positions + [0.04] * 2
        gripper_site_id = self.model.site("grip_site").id
        jac = np.zeros((6, self.model.nv))

        mujoco.mj_step(self.model, self.data, 1)
        current_pos = np.copy(self.data.site(gripper_site_id).xpos)
        current_mat = np.copy(self.data.site(gripper_site_id).xmat).reshape(3, 3)
        target_pos = current_pos + delta_pos
        return self.ik_trajectory_to_target_position(target_pos, start_joint_positions, num_points, verbose)
    

    def ik_trajectory_to_target_position(self, target_pos, start_joint_positions, num_points=100, verbose=True):
        # TODO: implement a version to reach a target rotation
        assert(len(start_joint_positions) == 7), "start_joint_positions should be a list of 7 elements"
        predicted_joints_seq = [np.array(start_joint_positions)]

        self.data.qpos[:] = start_joint_positions + [0.04] * 2
        gripper_site_id = self.model.site("grip_site").id
        jac = np.zeros((6, self.model.nv))

        mujoco.mj_step(self.model, self.data, 1)
        current_pos = np.copy(self.data.site(gripper_site_id).xpos)
        current_mat = np.copy(self.data.site(gripper_site_id).xmat).reshape(3, 3)
        mujoco.mj_jacSite(self.model, self.data, jac[:3], jac[3:], gripper_site_id)

        target_mat = current_mat

        if verbose:
            print("Current eef (pos): ", current_pos)
            print("Predicted goal eef (pos): ", target_pos)

        for i in trange(num_points):
            pos = current_pos + (target_pos - current_pos) * (i + 1) / (num_points)
            predicted_joints = self.inverse_kinematics(self.model, self.data, target_mat, pos, start_joint_positions)
            predicted_joints_seq.append(predicted_joints)
        debug_info = {
            "predicted_joints_seq": predicted_joints_seq,
            "target_pos": target_pos
        }
        return predicted_joints_seq, debug_info
    
    def simulate_joint_sequence(self, joint_sequence, loop=False, fps=30, render=True):

        recorded_pos = []
        recorded_mat = []

        if render:
            with mujoco.viewer.launch_passive(
                    model=self.model,
                    data=self.data,
                    show_left_ui=False,
                    show_right_ui=False,
                ) as viewer:
                    mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
                    while viewer.is_running():
                        for joint_conf in joint_sequence:
                            self.data.qpos[:] = joint_conf.tolist() + [0.04] * 2
                            mujoco.mj_step(self.model, self.data, 1)
                            viewer.sync()
                            time.sleep(1/fps)
                            gripper_site_id = self.model.site("grip_site").id
                            
                            # mujoco.mj_fwdPosition(model, data)
                            current_pos = np.copy(self.data.site(gripper_site_id).xpos)
                            current_mat = np.copy(self.data.site(gripper_site_id).xmat).reshape(3, 3)
                            recorded_pos.append(current_pos)
                            recorded_mat.append(current_mat)
                        if not loop:
                            break   
        else:
            for joint_conf in joint_sequence:
                self.data.qpos[:] = joint_conf.tolist() + [0.04] * 2
                mujoco.mj_step(self.model, self.data, 1)
                gripper_site_id = self.model.site("grip_site").id
                # mujoco.mj_fwdPosition(model, data)
                current_pos = np.copy(self.data.site(gripper_site_id).xpos)
                current_mat = np.copy(self.data.site(gripper_site_id).xmat).reshape(3, 3)
                recorded_pos.append(current_pos)
                recorded_mat.append(current_mat)
        recorded_pos = np.array(recorded_pos)
        recorded_mat = np.array(recorded_mat)
        return recorded_pos, recorded_mat

    def inverse_kinematics(self, model, data, target_mat, target_pos, reset_joint_positions):
        dtype = data.qpos.dtype
        jac = np.empty((6, model.nv), dtype=dtype)
        err = np.empty(6, dtype=dtype)

        jac_pos, jac_rot = jac[:3], jac[3:]
        err_pos, err_rot = err[:3], err[3:]
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, target_mat.reshape(9, 1))

        update_nv = np.zeros(model.nv, dtype=dtype)

        site_xquat = np.empty(4, dtype=dtype)
        neg_site_xquat = np.empty(4, dtype=dtype)
        error_rot_quat = np.empty(4, dtype=dtype)
        data.qpos[:] = reset_joint_positions + [0.04] * 2

        mujoco.mj_fwdPosition(model, data)
        gripper_site_id = model.site("grip_site").id

        site_xpos = data.site(gripper_site_id).xpos
        site_xmat = data.site(gripper_site_id).xmat

        max_steps = 100
        rot_weight = 1.0
        regularization_threshold = 0.1
        regularization_strength = 3e-2
        max_update_norm = 2.0
        progress_thresh = 20.0
        for steps in range(max_steps):
            err_norm = 0.0
            err_pos[:] = target_pos - site_xpos
            err_norm += np.linalg.norm(err_pos)

            mujoco.mju_mat2Quat(site_xquat, site_xmat)
            mujoco.mju_negQuat(neg_site_xquat, site_xquat)
            mujoco.mju_mulQuat(error_rot_quat, target_quat, neg_site_xquat)
            mujoco.mju_quat2Vel(err_rot, error_rot_quat, 1)
            err_norm += np.linalg.norm(err_rot) * rot_weight

            if err_norm < 1e-10:
                break

            else:
                mujoco.mj_jacSite(
                    model, data, jac_pos, jac_rot, gripper_site_id
                )
                jac_joints = jac
                reg_strength = (
                    regularization_strength if err_norm > regularization_threshold
                    else 0.0)
                update_joints = self.nullspace_method(
                    jac_joints, err, regularization_strength=reg_strength)
                update_norm = np.linalg.norm(update_joints)

                progress_criterion = err_norm / update_norm
                if progress_criterion > progress_thresh:
                    break
                if update_norm > max_update_norm:
                    update_joints *= max_update_norm / update_norm

                # Write the entries for the specified joints into the full `update_nv`
                # vector.

                mujoco.mj_integratePos(model, data.qpos, update_joints, 1)
                mujoco.mj_fwdPosition(model, data)

        qpos = data.qpos.copy()
        return qpos[:7]

    def nullspace_method(self, jac_joints, delta, regularization_strength=0.0):
        hess_approx = jac_joints.T.dot(jac_joints)
        joint_delta = jac_joints.T.dot(delta)
        if regularization_strength > 0:
            # L2 regularization
            hess_approx += np.eye(hess_approx.shape[0]) * regularization_strength
            return np.linalg.solve(hess_approx, joint_delta)
        else:
            return np.linalg.lstsq(hess_approx, joint_delta, rcond=-1)[0]

    def interpolate_dense_traj(self, joint_seq, minimal_displacement=0.005):
        new_joint_seq = []
        for i in range(len(joint_seq) - 1):
            new_joint_seq = new_joint_seq + [joint_seq[i]]
            max_joint_displacement = np.max(np.abs(joint_seq[i] - joint_seq[i + 1]))
            if max_joint_displacement < minimal_displacement:
                continue
            else:
                num_points = int(max_joint_displacement / minimal_displacement)
                for k in range(num_points):
                    new_joint = joint_seq[i] + (joint_seq[i + 1] - joint_seq[i]) * (k + 1) / (num_points + 1)
                    new_joint_seq.append(new_joint)
        new_joint_seq.append(joint_seq[-1])
        print("increased joint sequence from {} to {}".format(len(joint_seq), len(new_joint_seq)))
        return new_joint_seq
