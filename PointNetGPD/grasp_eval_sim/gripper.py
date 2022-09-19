import math
import os
import time
from matplotlib import cm

import numpy as np
import pybullet as p

from gripper_module import load_gripper
from misc.urdf_editor import UrdfEditor

import matplotlib
import matplotlib.pyplot as plt
import cv2

from pybullet_planning import CLIENT
from pybullet_planning import load_pybullet
from pybullet_planning import set_pose
from pybullet_planning import get_joint_state, get_num_joints, get_joint_info

class Gripper(object):
    
    """
    A moving mount and a gripper.
    the mount has 4 joints:
        0: prismatic x;
        1: prismatic y;
        2: prismatic z;
        3: revolute z;
    the gripper is defined by the `gripper_type`.
    """
    
    def __init__(self, gripper_type, home_position, voxel_size=0.004, trunc_margin_scale=5, **kwargs):
        self._gripper_type = gripper_type
        self._gripper_size = kwargs['gripper_size']
        self._home_position = home_position
        self._default_orientation = [0,0,0]
        # self._num_side_images = num_side_images
        
        self._gripper = load_gripper(gripper_type)(**kwargs)
        gripper_body_id = self._gripper.load(self._home_position)
        
        mount_urdf = 'assets/gripper/mount.urdf'
        mount_body_id = p.loadURDF(
            mount_urdf,
            basePosition=self._home_position,
            useFixedBase=True,
            physicsClientId=CLIENT
        )
        
        ed_mount = UrdfEditor()
        ed_mount.initializeFromBulletBody(mount_body_id, CLIENT)
        ed_gripper = UrdfEditor()
        ed_gripper.initializeFromBulletBody(gripper_body_id, CLIENT)
        
        self._gripper_parent_index = 6
        newjoint = ed_mount.joinUrdf(
            childEditor=ed_gripper,
            parentLinkIndex=self._gripper_parent_index,
            jointPivotXYZInParent=self._gripper.get_pos_offset(),
            jointPivotRPYInParent=p.getEulerFromQuaternion(self._gripper.get_orn_offset(), physicsClientId=CLIENT),
            jointPivotXYZInChild=[0, 0, 0],
            jointPivotRPYInChild=[0, 0, 0],
            parentPhysicsClientId=CLIENT,
            childPhysicsClientId=CLIENT
        )
        
        newjoint.joint_type = p.JOINT_FIXED
        newjoint.joint_name = "joint_mount_gripper"
        urdfname = f".tmp_combined_{self._gripper_type}_{self._gripper_size:.4f}_{np.random.random():.10f}_{time.time():.10f}.urdf"
        ed_mount.saveUrdf(urdfname)
        
        p.removeBody(mount_body_id, physicsClientId=CLIENT)
        p.removeBody(gripper_body_id, physicsClientId=CLIENT)
        
        self._body_id = load_pybullet(
            urdfname,
            fixed_base=True
        )
        set_pose(self._body_id, (self._home_position, p.getQuaternionFromEuler([0,0,0])))
        
        os.remove(urdfname)
        
        self._gripper.configure(self._body_id, self._gripper_parent_index+1)
        
        self._force = 10000
        # self._speed = 0.005
        self._speed = 0.004
        
        self.fix_joints(range(get_num_joints(self._body_id)))
        
        #############*
        # print('number of new joints', get_num_joints(self._body_id))
        # for i in range(get_num_joints(self._body_id)):
        #     print('joint info: ', get_joint_info(self._body_id, i))
        #     print('\n')
        # exit()
        ###########*
        
        
    def fix_joints(self, joint_ids):
        
        # current_states = np.array([p.getJointState(self._body_id, joint_id, physicsClientId=CLIENT)[0] for joint_id in joint_ids])
        current_states = np.array([get_joint_state(self._body_id, joint_id).jointPosition for joint_id in joint_ids])
        p.setJointMotorControlArray(
            self._body_id,
            joint_ids,
            p.POSITION_CONTROL,
            targetPositions=current_states,
            forces=[self._force] * len(joint_ids),
            positionGains=[self._speed] * len(joint_ids),
            physicsClientId=CLIENT
        )
        
    def open(self, open_scale):
        self._gripper.open(self._body_id, self._gripper_parent_index+1, open_scale=open_scale)
        
    def close(self):
        self._gripper.close(self._body_id, self._gripper_parent_index+1)
        # p.changeDynamics(self._body_id, 3, angularDamping=0.0001)
        
                
                
    def move(self, target_position, rotation_angle, stop_at_contact=False):
        """
        :param target_position: (x, y, z). the position of the bottom center, not the base!
        :param rotation_angle: rotation in z axis \in [0, 2 * \pi]. For 2-finger gripper, angle=0 --> parallel to x-axis
        """
        
        target_position = np.array(target_position) - np.array(self._home_position)
        # target_position = np.array(target_position)
        joint_ids = [0, 1, 2, 5]
        # joint_ids = [0, 1, 2]
        target_states = [target_position[0], target_position[1], target_position[2], rotation_angle%(2*np.pi)]
        # print(target_states)
        
        # p.changeDynamics(self._body_id, 3, angularDamping=0.001)
        
        
        p.setJointMotorControlArray(
            self._body_id,
            joint_ids,
            p.POSITION_CONTROL,
            targetPositions=target_states, 
            forces=[self._force] * len(joint_ids),
            # positionGains=[self._speed] * len(joint_ids),
            positionGains=[0.01] * len(joint_ids),
            physicsClientId=CLIENT
        )
        
        # p.setJointMotorControl2(
        #     self._body_id,
        #     3,
        #     p.POSITION_CONTROL,
        #     targetPosition=rotation_angle%(2*np.pi), 
        #     force=self._force,
        #     positionGain=0.01,
        #     # velocityGain=1,
        #     physicsClientId=CLIENT
        # )
        
        for i in range(240 * 6):
            current_states = np.array([p.getJointState(self._body_id, joint_id, physicsClientId=CLIENT)[0] for joint_id in joint_ids])
            states_diff = np.abs(target_states - current_states)
            # stop moving gripper if gripper collide with other objects
            if stop_at_contact:
                is_in_contact = False
                points = p.getContactPoints(bodyA=self._body_id, physicsClientId=CLIENT)
                if len(points) > 0:
                    for ps in points:
                        if ps[9] > 0:
                            is_in_contact = True
                            break
                if is_in_contact:
                    break
            if np.all(states_diff < 1e-4):
                break
            self._gripper.step_constraints(self._body_id, self._gripper_parent_index+1)
            
            if self.is_gripper_broken():
                self.remove()
                break
            
            time.sleep(1/480)
            p.stepSimulation()
            

        self.fix_joints(joint_ids)
    
    def remove(self):
        p.removeBody(self._body_id)
        
    def is_gripper_broken(self):
        joint_ids = [0, 1, 2, 3]
        current_states = np.array([get_joint_state(self._body_id, joint_id).jointPosition for joint_id in joint_ids])
        return any(np.isnan(current_states))
