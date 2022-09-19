import imp
from json import load
from pathlib import Path
import os
from pickle import DUP

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

import torch
from timeit import TimeIt

from gripper import Gripper

import pybullet as p
from pybullet_planning import load_pybullet, connect, enable_gravity, add_data_path, create_plane, \
 set_point, remove_body, get_pose, create_obj, set_pose
from pybullet_planning import get_joint_state
from pybullet_planning import RED, BLUE, GREEN, GREY, CLIENT
from pybullet_planning import set_camera_pose, get_image, get_camera

import threading

from camera import Camera
from dual_grasp_sim import DualGraspSim

def read_grasp_file(grasp):
    center_point = grasp[0:3]
    major_pc = grasp[3:6] #* binormal
    width = grasp[6]
    angle = grasp[7]
    
    cos_t = np.cos(angle)
    sin_t = np.sin(angle)
    R1 = np.c_[[cos_t, 0, sin_t],[0, 1, 0],[-sin_t, 0, cos_t]]
    axis_y = major_pc
    axis_x = np.array([axis_y[1], -axis_y[0], 0])
    if np.linalg.norm(axis_x) == 0:
        axis_x = np.array([1, 0, 0])
    axis_x = axis_x / np.linalg.norm(axis_x)
    axis_y = axis_y / np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    R2 = np.c_[axis_x, np.c_[axis_y, axis_z]]
    approach_normal = R2.dot(R1)[:, 0]
    approach_normal = approach_normal / np.linalg.norm(approach_normal)
    minor_pc = np.cross(major_pc, approach_normal)
    
        


def main():
    
    connect(use_gui=True)
    shapenet_object = '/home/kangmin/Workspace/dual-arm-grasp-gen/PointNetGPD/PointNetGPD/data/shapenetcore/rexchair/1d7fdf837564523dc89a28b5e6678e0/watertight_model.obj'
    generated_grasp = '/home/kangmin/Workspace/dual-arm-grasp-gen/PointNetGPD/dex-net/apps/generated_grasps/1d7fdf837564523dc89a28b5e6678e0.npy'
    
    
    sim = DualGraspSim(shapenet_object)
    sim.step_simulation(generated_grasp)
    

if __name__ == '__main__':
    main()