"""Example script of moving robot joint positions."""
import os
import sys
# print(os.path.dirname(os.path.abspath(__file__)))
# exit()

sys.path.insert(0, '/home/ani/franka_control_ws/deoxys_control/franka_async_controller/build')
import franka_controller as fc
import numpy as np
config = fc.KinematicsConfig()
config.verbose = True
config.urdf_path = "/home/ani/franka_control_ws/deoxys_control/franka_async_controller/fr3_robot.urdf"

# kin = fc.FrankaKinematics("172.16.0.2", config)
kin = fc.FrankaKinematics(config)                                                                                                                                      
kin.initialize()

# q = np.array([0.09162008114028396, -0.19826458111314524, -0.01990020486871322,-2.4732269941140346, -0.01307073642274261, 2.30396583422025, 0.8480939705504309])
# pose_0 = kin.compute_fk(q)              # 返回 4x4 numpy array                                                                                                                         
# print("EE position:", pose_0[:3, 3])                                                                                                                                                   
# print("EE rotation:\n", pose_0[:3, :3])

# # """
# #     Current Pose: [[ 0.99946286  0.01816329  0.02727742  0.45748515]
# #                     [ 0.0182672  -0.99982677 -0.00356477  0.03220093]
# #                     [ 0.02720795  0.00406114 -0.99962155  0.26492077]
# #                     [ 0.          0.          0.          1.        ]]
# #     """

# # target_pose = np.array([[ 0.99946286,  0.01816329,  0.02727742,  0.45748515],
# #                         [ 0.0182672,  -0.99982677, -0.00356477,  0.03220093],
# #                         [ 0.02720795, 0.00406114, -0.99962155,  0.26492077],
# #                         [ 0. ,         0. ,         0. ,        1.        ]])                                                                                                                                                              
# # # target_pose[:3, 3] = [0.45748515, 0.03220093, 0.26492077]

# seed = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])                                                                                                                       
# q_result = kin.solve_ik(pose_0, seed)  # 返回 np.array(7,) 或 None
# print("IK result:", q_result)

# pose_1 = kin.compute_fk(q_result)              # 返回 4x4 numpy array                                                                                                                         
# print("EE position:", pose_1[:3, 3])                                                                                                                                                   
# print("EE rotation:\n", pose_1[:3, :3])

# ##
# from deoxys.utils import YamlConfig, transform_utils
# def pose_mat_to_vec(pose_mat):
#     pos = pose_mat[:3, 3].tolist()
#     quat = transform_utils.mat2quat(pose_mat[:3, :3]).tolist()
#     return pos + quat


# def compute_errors(pose_1, pose_2):

#     pose_a = (
#         pose_1[:3]
#         + transform_utils.quat2axisangle(np.array(pose_1[3:]).flatten()).tolist()
#     )
#     pose_b = (
#         pose_2[:3]
#         + transform_utils.quat2axisangle(np.array(pose_2[3:]).flatten()).tolist()
#     )
#     return np.abs(np.array(pose_a) - np.array(pose_b))
# ##

# pose0_vec = pose_mat_to_vec(pose_0)
# pose1_vec = pose_mat_to_vec(pose_1)
# errors = compute_errors(pose0_vec, pose1_vec)
# print("Pose error [dx, dy, dz, dax, day, daz]:", errors)
# print("Position error norm:", np.linalg.norm(errors[:3]))
# print("Orientation error norm:", np.linalg.norm(errors[3:]))

# exit()

import argparse
import pickle
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt


from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig, transform_utils
from deoxys.utils.input_utils import input2action
from deoxys.experimental.motion_utils import reset_joints_to, move_joints_to
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-position-controller.yml"
    )
    args = parser.parse_args()
    return args


def pose_mat_to_vec(pose_mat):
    pos = pose_mat[:3, 3].tolist()
    quat = transform_utils.mat2quat(pose_mat[:3, :3]).tolist()
    return pos + quat


def compute_errors(pose_1, pose_2):

    pose_a = (
        pose_1[:3]
        + transform_utils.quat2axisangle(np.array(pose_1[3:]).flatten()).tolist()
    )
    pose_b = (
        pose_2[:3]
        + transform_utils.quat2axisangle(np.array(pose_2[3:]).flatten()).tolist()
    )
    return np.abs(np.array(pose_a) - np.array(pose_b))


def main():
    args = parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(config_root + f"/{args.controller_cfg}").as_easydict()

    controller_type = "JOINT_POSITION"

    # Golden resetting joints
    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
    ]

    """
    Current Pose: [[ 0.99946286  0.01816329  0.02727742  0.45748515]
                    [ 0.0182672  -0.99982677 -0.00356477  0.03220093]
                    [ 0.02720795  0.00406114 -0.99962155  0.26492077]
                    [ 0.          0.          0.          1.        ]]
    """

    reset_joints_to(robot_interface, reset_joint_positions)

    # move_joints_to(robot_interface, reset_joint_positions)
    # reset_joints_to(robot_interface, reset_joint_positions)

    pose_mat_initial = robot_interface.last_eef_pose.copy()
    print("Current Pose INITIAL:", pose_mat_initial)

    for _ in range(5):
        pose_mat = robot_interface.last_eef_pose.copy()
        print("Current Pose:", pose_mat)

        target_pose = pose_mat.copy()
        target_pose[:3, 3] += np.array([0.05, 0.0, 0.0])
        print("Target Pose:", target_pose)

        ### Solve IK
        seed = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])                                                                                                                       
        q_result = kin.solve_ik(target_pose, seed)
        ###
        move_joints_to(robot_interface, q_result)

        pose_mat_1 = robot_interface.last_eef_pose.copy()
        print("Current Pose:", pose_mat_1)

        target_pose_1 = pose_mat_1.copy()
        target_pose_1[:3, 3] -= np.array([0.05, 0.0, 0.0])
        print("Target Pose:", target_pose_1)

        ### Solve IK                                                                                                                    
        q_result_1 = kin.solve_ik(target_pose_1, seed)
        ###
        move_joints_to(robot_interface, q_result_1)


    pose_mat_end = robot_interface.last_eef_pose.copy()
    print("Current Pose END:", pose_mat_end)

    # target_pose_o = pose_mat_1.copy()
    # target_pose_o[:3, 3] -= np.array([0.03, 0.0, 0.0])


    # ###
    # seed = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])                                                                                                                       
    # q_result = kin.solve_ik(target_pose_o, seed)
    # ###

    # reset_joints_to(robot_interface, q_result)
    # pose_mat_2 = robot_interface.last_eef_pose.copy()
    # print("Current Pose:", pose_mat_2)


    pose0_vec = pose_mat_to_vec(pose_mat_initial)
    pose1_vec = pose_mat_to_vec(pose_mat_end)
    errors = compute_errors(pose0_vec, pose1_vec)
    print("Pose error [dx, dy, dz, dax, day, daz]:", errors)
    print("Position error norm:", np.linalg.norm(errors[:3]))
    print("Orientation error norm:", np.linalg.norm(errors[3:]))

    exit()

    # This is for varying initialization of joints a little bit to
    # increase data variation.
    reset_joint_positions = [
        e + np.clip(np.random.randn() * 0.005, -0.005, 0.005)
        for e in reset_joint_positions
    ]
    action = reset_joint_positions + [-1.0]

    print(f"{len(robot_interface._state_buffer)=}")

    i = 0
    while True:
        if len(robot_interface._state_buffer) > 0:
            logger.info(f"Current Robot joint: {np.round(robot_interface.last_q, 3)}")
            logger.info(f"Desired Robot joint: {np.round(robot_interface.last_q_d, 3)}")

            if (
                np.max(
                    np.abs(
                        np.array(robot_interface._state_buffer[-1].q)
                        - np.array(reset_joint_positions)
                    )
                )
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg,
        )
        i += 1
        print(i)
    robot_interface.close()


if __name__ == "__main__":
    main()
