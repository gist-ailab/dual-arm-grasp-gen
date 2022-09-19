import pybullet as p
import numpy as np
from pybullet_planning.utils import CLIENT

class Camera:
    def __init__(self, cam_pos, cam_tar, cam_up_vector, near, far, size, fov):
        self.width, self.height = size
        self.near = near
        self.far = far
        self.fov = fov

        aspect = float(self.width / self.height)
        self.view_matrix = p.computeViewMatrix(cam_pos, cam_tar, cam_up_vector)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, self.near, self.far)

        _view_matrix = np.array(self.view_matrix).reshape((4, 4), order='F')
        _projection_matrix = np.array(self.projection_matrix).reshape((4, 4), order='F')
        self.tran_pix_world = np.linalg.inv(_projection_matrix @ _view_matrix)


    def shot(self):
        # Get depth values using the OpenGL renderer
        _w, _h, rgb, depth, seg = p.getCameraImage(self.width, self.height,
                                                   self.view_matrix, self.projection_matrix,
                                                   shadow=False,
                                                   flags=0,
                                                   renderer=p.ER_BULLET_HARDWARE_OPENGL,# p.ER_TINY_RENDERER
                                                   physicsClientId=CLIENT
                                                   )
        # rgb = rgb.reshape((self.height, self.width, -1))[:, :, :3]
        # depth = depth.reshape((self.height, self.width))
        
        return rgb, depth, seg

    def rgbd_2_world(self):
        _, depth, _ = self.shot()
        x = (2 * np.arange(0, self.width) - self.width) / self.width
        x = np.repeat(x[None, :], self.height, axis=0)
        y = -(2 * np.arange(0, self.height) - self.height) / self.height
        y = np.repeat(y[:, None], self.width, axis=1)
        z = 2 * depth - 1

        pix_pos = np.array([x.flatten(), y.flatten(), z.flatten(), np.ones_like(z.flatten())]).T
        position = self.tran_pix_world @ pix_pos.T
        position = position.T
        position[:, :] /= position[:, 3:4]

        return position[:, :3].reshape(*x.shape, -1)
        
        # # x = (2 * w - self.width) / self.width
        # # y = -(2 * h - self.height) / self.height
        # # z = 2 * d - 1
        # # pix_pos = np.array((x, y, z, 1))
        # # position = self.tran_pix_world @ pix_pos
        # # position /= position[3]

        # return position[:3]
    
    def rgbd_2_world_batch(self, depth):
        # reference: https://stackoverflow.com/a/62247245
        
        _, depth, _ = self.shot()
        x = (2 * np.arange(0, self.width) - self.width) / self.width
        x = np.repeat(x[None, :], self.height, axis=0)
        y = -(2 * np.arange(0, self.height) - self.height) / self.height
        y = np.repeat(y[:, None], self.width, axis=1)
        z = 2 * depth - 1

        pix_pos = np.array([x.flatten(), y.flatten(), z.flatten(), np.ones_like(z.flatten())]).T
        position = self.tran_pix_world @ pix_pos.T
        position = position.T
        # print(position)

        position[:, :] /= position[:, 3:4]

        return position[:, :3].reshape(*x.shape, -1)