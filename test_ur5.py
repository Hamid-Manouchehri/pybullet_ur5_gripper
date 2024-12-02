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

# create the environment
env = ur5GymEnv(renders=True, maxSteps=120, useIK=True,
        actionRepeat=1, task=0, randObjPos=False,
        simulatedGripper=False, learning_param=0.05)

robot_id = env.ur5


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
    # print("joint ", joint_index, " error: ", error, " desired angle: ", desired_angle, "current angle: ", current_angle)
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

    t = 0
    time_step = .001  # TODO
    trailDuration = 15
    prevPose=[0,0,0]
    prevPose1=[0,0,0]
    hasPrevPose = 0

    while 1:

        t = t + time_step

        pos = [0.817 - t, 0.234, 0.063 + t]
        init_end_effector_orient = [-np.pi, 0., np.pi/2]  # Euler angles [roll, pitch, yaw]

        for i in range (1):
            
            
            all_joint_angles = env.calculate_ik(pos, init_end_effector_orient)
            ur5_joint_angles = all_joint_angles[:6]  # Use the first 6 joints for UR5

            env.set_joint_angles(ur5_joint_angles)
            p.stepSimulation()

        ls = p.getLinkState(robot_id, eff_index)	
        if (hasPrevPose):
            p.addUserDebugLine(prevPose,pos,[0,0,0.3],1,trailDuration)
            p.addUserDebugLine(prevPose1,ls[4],[1,0,0],1,trailDuration)
        prevPose=pos
        prevPose1=ls[4]
        hasPrevPose = 1	


if __name__ == '__main__':
    main()