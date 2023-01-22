from .SCALER_v2_Leg_6DOF_gripper import Leg
import numpy as np


def GetFootstepPosition(w_T_b, body_angle, joint_angles):
    #input
    # w_T_b : 4x4 transformation matrix from world to body
    # body_angle: body motor angle
    # joint_angles: list of 12 joint angles [should_angle, q11, q12]*4 for 4 legs

    #output
    #list of 4 4x4 matrix for 4 wrist positions in world frame
 
    k_model = Leg()
    fk_res = []
    for i in range(4):
        res = k_model.leg_fk_direct_calculation(body_angle, [joint_angles[i*3], joint_angles[1+i*3], joint_angles[2+i*3], 0 ,0 ,0], i, which_link=11, use_quaternion = False)
        res = np.dot(w_T_b, res)
        fk_res.append(res)
    return fk_res


"""
#for test    
w_T_b = np.eye(4, dtype = np.float32)
body_angle = 0
joint_angles = [0,0,np.pi/2,0,0,np.pi/2,0,0,np.pi/2,0,0,np.pi/2]

print(GetFootstepPosition(w_T_b, body_angle, joint_angles))
"""


