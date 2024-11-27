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

env = ur5GymEnv(renders=True, maxSteps=120, useIK=False,
        actionRepeat=1, task=0, randObjPos=False,
        simulatedGripper=False, learning_param=0.05)

obs = env.reset()
data_size = obs.shape[0]

def main():
    
    positions = [0.0, 0.0, 0.0, -0.0, -0.0, 0.0]  # gripper: 1.0 (close), -1.0 (open)
    state = env.reset()
    ep_reward = 0

    robot_id = env.ur5


    # for i in range(300):
    #     for t in range(1, 120):

    #         state, reward, env_done, info = env.step(positions)  

    joint_angles = env.calculate_ik([1,2,3], [0.0, 1/2*math.pi, 0.0])
    print(joint_angles)

    while True:

        positions = [0.0, -0.5, -0.3, -0.0, 0.0, 0.0]  # gripper: 1.0 (close), -1.0 (open)
        env.set_joint_angles(positions) 
        gripper_action = -1
        gripper_action = np.clip(gripper_action/2.5, -0.4, 0.4)
        env.control_gripper(gripper_action)

        p.stepSimulation()
        time.sleep(.05)


if __name__ == '__main__':
    main()

