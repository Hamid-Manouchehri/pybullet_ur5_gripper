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

def get_args():
    parser = ArgumentParser(description=title)
    arg = parser.add_argument
    # trained model file for demo:
    # arg('--inputfile', type=str, help='input model file (.pth)') # trained model
    # env:
    arg('--seed', type=int, default=543, help='random seed (default: 543)')
    arg('--mel', type=int, default=120, help='max episode length')
    arg('--repeat', type=int, default=1, help='repeat action')
    arg('--render', action='store_true', default=True, help='render the environment')
    arg('--randObjPos', action='store_true', default=False, help='fixed object position to pick up')
    arg('--useIK', action='store_true', default=False, help='use IK or direct control')
    # sim:
    # arg('--data_size', type=int, default=30, help='dataset data size')
    arg('--lp', type=float, default=0.05, help='learning parameter for task')
    arg('--task', type=int, default=0, help='task to learn: 0 move, 1 pick-up, 2 drop')
    arg('--simgrip', action='store_true', default=False, help='simulated gripper')
    # dataset:
    arg('--dataset', action='store_true', default=False, help='create a dataset for imitation learning')
    arg('--ds', type=int, default=1000, help='number of episodes to collect for dataset')
    args = parser.parse_args()
    return args

args = get_args() # all input arguments

np.set_printoptions(precision=2, suppress=True)
torch.set_printoptions(profile="full", precision=2)

# create the environment

env = ur5GymEnv(renders=args.render, maxSteps=args.mel, useIK=args.useIK,
        actionRepeat=args.repeat, task=args.task, randObjPos=args.randObjPos,
        simulatedGripper=args.simgrip, learning_param=args.lp)

obs = env.reset()
args.data_size = obs.shape[0]

def main():
    
    positions = [0.0, 0.0, 0.0, -0.0, -0.0, 0.0, 0.0]  # gripper: 1.0 (close), -1.0 (open)
    state = env.reset()
    ep_reward = 0

    # print("inverse kinematics: \n\n\n", env.calculate_ik([0, -1.57, 0, 0, 0, 0], [0, 0, 0]))

    for i in range(300):
        for t in range(1, args.mel):

            state, reward, env_done, info = env.step(positions)  

            ep_reward += reward

    while True:
        p.stepSimulation()


if __name__ == '__main__':
    main()

