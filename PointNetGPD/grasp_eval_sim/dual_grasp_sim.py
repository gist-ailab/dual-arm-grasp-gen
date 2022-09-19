from pathlib import Path
import os

import time
import math
from tkinter.tix import Tree
import numpy as np
from scipy.spatial import distance
import random

from matplotlib import pyplot as plt
from tqdm import tqdm

import cv2
from cv2 import circle
# from dual_gripper.grasp_data_gen import wait_till_stable

import torch
from timeit import TimeIt

from gripper import Gripper

import pybullet as p
from pybullet_planning import load_pybullet, connect, enable_gravity, add_data_path, create_plane, \
 set_point, remove_body, get_pose, create_obj, set_pose, disable_gravity
from pybullet_planning import get_joint_state
from pybullet_planning import RED, BLUE, GREEN, GREY, CLIENT
from pybullet_planning import set_camera_pose, get_image, get_camera

import threading

from camera import Camera

class DualGraspSim():
    def __init__(self, object, gripper1=None, gripper2=None) -> None:
        self.object = create_obj(object, mass=1)
        gripper_kwargs = dict()
        gripper_kwargs['gripper_size'] = 1.0
        
        generated_grasp = '/home/kangmin/Workspace/dual-arm-grasp-gen/PointNetGPD/dex-net/apps/generated_grasps/1d7fdf837564523dc89a28b5e6678e0.npy'
        g = np.load(generated_grasp)
        g_good = g[g[:, -2] <= 0.6]
        g_good = g_good[np.random.choice(len(g_good), size=1, replace=True)]
        g_good = np.squeeze(g_good, axis=0)
        gripper_pos = g_good[0:3]
        
        gripper1 = Gripper(
            gripper_type='robotiq_2f_85',
            home_position=gripper_pos,
            **gripper_kwargs
        )
        self.gripper1 = gripper1
        self.gripper2 = gripper2
        
        add_data_path()
        disable_gravity()
        # load_pybullet('plane.urdf')
        
        # set_pose(self.object, ([0,0,0.5],[0.7071068, 0, 0, 0.7071068]))
        # set_pose(self.object, ([0,0,0],[0,0,0]))
        
        
    def step_simulation(self, generated_grasp):
        # g = np.load(generated_grasp)
        # g_good = g[g[:, -2] <= 0.6]
        # g_good = g_good[np.random.choice(len(g_good), size=1, replace=True)]
        # g_good = np.squeeze(g_good, axis=0)
        # gripper_pos = g_good[0:3]
        # print(gripper_pos)
        # set_point(self.gripper1._body_id, gripper_pos)
        for i in range(10000):
            p.stepSimulation()
            time.sleep(1/120)
            