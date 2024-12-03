'''
Hamid Manouchehri
Nov, 2024

Optimization project, in hand manipulation of object.
grasping an object in such a way to determine COM by handling object.

reference: https://github.com/culurciello/pybullet_ur5_gripper, 
           https://github.com/josepdaniel/UR5Bullety
'''
#!/usr/bin/env python3

import numpy as np
from gym_env import ur5GymEnv
import pybullet as p 
import time

# create the environment
env = ur5GymEnv(renders=True, maxSteps=120, useIK=True,
        actionRepeat=1, task=0, randObjPos=False,
        simulatedGripper=False, learning_param=0.05)

robot_id = env.ur5

eff_index = 7  # End-effector index


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

    if abs(error) > angle_threshold:
        velocity = np.clip(error, -desired_velocity, desired_velocity)

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
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index + 1,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0.0,
            force=max_force
        )
        return True  # Joint reached the target
    

def generate_trajectory(start_pos, end_pos, start_orient, end_orient, num_points):
    """
    Generates a Cartesian trajectory for position and orientation.

    Parameters:
        start_pos (list): Starting Cartesian position [x, y, z].
        end_pos (list): Ending Cartesian position [x, y, z].
        start_orient (list): Starting orientation in Euler angles [roll, pitch, yaw].
        end_orient (list): Ending orientation in Euler angles [roll, pitch, yaw].
        num_points (int): Number of trajectory points.

    Returns:
        tuple: Interpolated positions and orientations.
    """
    # Interpolate positions
    positions = np.linspace(start_pos, end_pos, num_points)
    orientations = np.linspace(start_orient, end_orient, num_points)  # Simplified for Euler angles

    return positions, orientations


def execute_trajectory(time_step, duration, start_pos, final_pos, start_orientation, final_orientation):

    t = 0
    position_tolerance = 0.01
    trailDuration = 15
    prevPose=[0,0,0]
    prevPose1=[0,0,0]
    hasPrevPose = 0

    while True:

        position_1 = position_path(t, duration, start_pos, final_pos)
        orientation_1 = orientation_path(t, duration, start_orientation, final_orientation)
        
        all_joint_angles = env.calculate_ik(position_1, orientation_1)
        ur5_joint_angles = all_joint_angles[:6]  # Use the first 6 joints for UR5

        env.set_joint_angles(ur5_joint_angles)

        ls = p.getLinkState(robot_id, eff_index)	
        if (hasPrevPose):
            p.addUserDebugLine(prevPose, position_1, [0,0,0.3], 1, trailDuration)
            p.addUserDebugLine(prevPose1, ls[4], [1,0,0], 1, trailDuration)
        prevPose=position_1
        prevPose1=ls[4]
        hasPrevPose = 1	

        t += time_step

        if np.linalg.norm(np.array(env.get_current_pose()[0]) - np.array(final_pos)) < position_tolerance:
            print("End-effector reached the target position.")
            print(f"Current position: {env.get_current_pose()[0]}, Target position: {final_orientation}")
            break

        p.stepSimulation()



def position_path(t, t_max, start_pos, end_pos):

    return start_pos + (end_pos - start_pos) * (t / t_max)



def orientation_path(t, t_max, start_orient, end_orient):
    """ orientation is in Euler. """

    return start_orient + (end_orient - start_orient) * (t / t_max)




def main():

    time_step = .1  # TODO

    start_pos = np.array([0.817, 0.234, 0.063])
    second_pos = np.array([0.468, 0.154, 0.481])
    third_pos = np.array([0.468, 0.154, 0.381])

    start_orientation = np.array([-np.pi, 0., np.pi / 2])
    second_orientation = np.array([-np.pi, 0., np.pi])
    third_orientation = np.array([-np.pi, np.pi/2, np.pi])

    duration = 20
    execute_trajectory(time_step, duration, start_pos, second_pos, start_orientation, second_orientation)
    execute_trajectory(time_step, duration, second_pos, third_pos, second_orientation, third_orientation)


    while True:
        # Keep running the simulator
        p.stepSimulation()
        time.sleep(0.01)



if __name__ == '__main__':
    main()