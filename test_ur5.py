# E. Culurciello
# February 2021

# PyBullet UR-5 from https://github.com/josepdaniel/UR5Bullety

#!/usr/bin/env python3

import numpy as np
from itertools import count
from collections import namedtuple
import time, math
from random import randint
import torch
from argparse import ArgumentParser
import gym
from gym_env import ur5GymEnv
import pybullet as p 

title = 'PyBullet UR5 robot'

np.set_printoptions(precision=2, suppress=True)
torch.set_printoptions(profile="full", precision=2)

# create the environment

env = ur5GymEnv(renders=True, maxSteps=120, useIK=True,
        actionRepeat=1, task=0, randObjPos=False,
        simulatedGripper=False, learning_param=0.05)

robot_id = env.ur5

# obs = env.reset()
# data_size = obs.shape[0]


def control_joint_velocity(robot_id, joint_index, desired_angle, desired_velocity, angle_threshold=0.01, max_force=150):
    """
    Controls the velocity of a specific joint to reach the desired angle dynamically.
    
    Parameters:
        robot_id (int): ID of the robot in PyBullet.
        joint_index (int): Index of the joint to control.
        desired_angle (float): Desired joint angle in radians.
        max_velocity (float): Maximum joint velocity in rad/s.
        angle_threshold (float): Threshold to stop the joint (default: 0.01 rad).
        max_force (float): Maximum torque/force to apply (default: 150).
    
    Returns:
        bool: True if the joint reached the desired angle within the threshold, False otherwise.
    """
    # Get the current angle of the joint
    current_angle = p.getJointState(robot_id, joint_index + 1)[0]
    
    # Calculate the error
    error = desired_angle - current_angle
    print("joint ", joint_index, " error: ", error, " desired angle: ", desired_angle, "current angle: ", current_angle)
    # print("error: ", joint_index, " ", error)

    if abs(error) > angle_threshold:
        # Dynamically calculate velocity based on the error
        velocity = np.clip(error, -desired_velocity, desired_velocity)
        # print("des vel: ", desired_velocity)
        # velocity = np.clip(error, abs(desired_velocity), -abs(desired_velocity))
        # print("vel if", joint_index, " ", velocity)


        # Set joint velocity
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index + 1,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=max_force
        )
        return False  # Joint has not yet reached the target
    else:
        # print("vel else:", joint_index, " ")
        # Stop the joint if within the threshold
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index + 1,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=max_force
        )
        return True  # Joint reached the target
    

def main():

    eff_index = 7  # End-effector index

    init_joint_angles = np.array([
    0.,                                         # shoulder_pan
    -np.pi/3,                                   # shoulder_lift
    np.pi/2,                                   # elbow
    -4*np.pi/6,                                 # wrist_1
    -np.pi/2 + .1,                              # wrist_2
    0.])                                        # wrist_3 (rad) TODO

    init_joint_vel = np.array([
    0.,                                         # shoulder_pan
    -3.,                                        # shoulder_lift
    2.,                                         # elbow
    3.,                                         # wrist_1
    3.,                                         # wrist_2
    0.])                                        # wrist_3 rad / sec, TODO


    reach_status = []
    while True:  # joint_control for inintial configuration of the robot

        for joint_index, _ in enumerate(init_joint_vel):
            reached = control_joint_velocity(
                robot_id=robot_id,
                joint_index=joint_index,
                desired_angle=init_joint_angles[joint_index],
                desired_velocity=init_joint_vel[joint_index],
                angle_threshold=0.1,
                max_force=150
            )
            reach_status.append(reached)

        # gripper_action = .4
        # gripper_action = np.clip(gripper_action/2.5, -0.4, 0.4)
        # env.control_gripper(gripper_action)

        if all(reach_status):
            print("\n\n\n initial configuration completed \n\n\n")
            break

        reach_status = []  # reset the list
        p.stepSimulation()
        time.sleep(0.05)

    end_effector_pos = [0.41, 0.0, .163]  # [x, y, z]
    end_effector_orient = [-np.pi, 0., np.pi/2]  # Euler angles [roll, pitch, yaw]

    # Calculate inverse kinematics for the target position
    all_joint_angles = env.calculate_ik(end_effector_pos, end_effector_orient)
    ur5_joint_angles = all_joint_angles[:6]  # Use the first 6 joints for UR5

    while True:
        # Command the robot to move to the calculated joint angles
        env.set_joint_angles(ur5_joint_angles)

        # Optional: Control the gripper
        gripper_action = -0.1  # [-0.4, 0.4] for open/close
        env.control_gripper(gripper_action)

        # Check if the end-effector is close enough to the target position
        current_pos = env.get_current_pose()[0]
        print("Current end-effector position:", current_pos)
        if np.allclose(current_pos, end_effector_pos, atol=1e-3):  # Tolerance of 1 mm
            print("Target position reached!")
            break

        # Step the simulation

        pose, orient = env.get_current_pose()
        print("end effector pose: ", pose, p.getEulerFromQuaternion(orient))
        p.stepSimulation()
        time.sleep(0.05)



if __name__ == '__main__':
    main()