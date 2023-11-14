import numpy as np

class cam_arm_consts:
    body_joint1_y_offset = 27.85
    body_joint1_z_offset = 55
    joint1_joint2_z_offset = 57.25
    joint2_joint3_x_offset = 201.46
    joint3_wrist1_x_offset = 201.46
    wrist1_cam_x_offset = 47.05
    T_body_joint1 = np.array([[ 1,0, 0,0],
                              [ 0,1, 0,body_joint1_y_offset],
                              [ 0,0, 1,body_joint1_z_offset],
                              [ 0,0, 0,1]])
    T_wrist1_cam = np.array([[ 0, 0, 1,wrist1_cam_x_offset],
                             [ 0,-1, 0,0],
                             [ 1, 0, 0,0],
                             [ 0, 0, 0,1]])                             
    