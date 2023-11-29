import numpy as np
from .cam_arm_hardware_constants import cam_arm_consts

body_joint1_y_offset = cam_arm_consts.body_joint1_y_offset
body_joint1_z_offset = cam_arm_consts.body_joint1_z_offset
joint1_joint2_z_offset = cam_arm_consts.joint1_joint2_z_offset
joint2_joint3_x_offset = cam_arm_consts.joint2_joint3_x_offset
joint3_wrist1_x_offset = cam_arm_consts.joint3_wrist1_x_offset
T_body_joint1 = cam_arm_consts.T_body_joint1
T_wrist1_cam = cam_arm_consts.T_wrist1_cam
T_joint1_body = np.linalg.inv(T_body_joint1)
wrist1_cam_x_offset = cam_arm_consts.wrist1_cam_x_offset

def Rotz(joint):
    return np.array([[ np.cos(joint), -np.sin(joint), 0,0],
                     [ np.sin(joint),  np.cos(joint), 0,0],
                     [             0,              0, 1,0],
                     [ 0,0, 0,1]]) 
                     
def Roty(joint):
    return np.array([[ np.cos(joint), 0, np.sin(joint),0],
                     [             0, 1,             0,0],
                     [-np.sin(joint), 0, np.cos(joint),0],
                     [ 0,0, 0,1]]) 

class Cam_arm:
    @staticmethod
    def cam_arm_fk(joints):
        #input:
        #joints : a list of [joint1, joint2, joint3, wrist1] in rad
        #output:
        #camera frame in scaler body frame
        joint1 = joints[0]
        joint2 = joints[1]
        joint3 = joints[2]
        wrist1 = joints[3]

        T_body_joint1_after = np.dot(T_body_joint1, Rotz(joint1))
    
        T_joint1_joint2 = np.dot(Rotz(np.pi/2), Roty(-np.pi/2))
        T_joint1_joint2[2,3] = joint1_joint2_z_offset
        
        T_body_joint2 = np.dot(T_body_joint1_after, T_joint1_joint2)
        T_body_joint2_after = np.dot(T_body_joint2, Rotz(joint2))
    
        T_joint2_joint3 = np.eye(4, dtype=np.float32)
        T_joint2_joint3[0,3] = joint2_joint3_x_offset
        T_body_joint3 = np.dot(T_body_joint2_after, T_joint2_joint3)
        T_body_joint3_after = np.dot(T_body_joint3, Rotz(joint3))
    
        T_joint3_wrist1 = np.eye(4, dtype=np.float32)
        T_joint3_wrist1[0,3] = joint3_wrist1_x_offset
        T_body_wrist1 = np.dot(T_body_joint3_after, T_joint3_wrist1)
        T_body_wrist1_after = np.dot(T_body_wrist1, Rotz(wrist1))
    
        T_body_cam = np.dot(T_body_wrist1_after, T_wrist1_cam)
    
        return T_body_cam

    @staticmethod
    def cam_arm_ik(Px_b,Py_b,Pz_b,cam_angle):
        #input:
        #Px,Py,Pz : (Px,Py,Pz) position of camera frame in scaler body frame
        #cam_angle : camera z axis angle to the negative z axis of the body, it is equal to joint2 + joint3 + wrist1
        #output:
        #[joint1, joint2, joint3, wrist1] angle 
        #Note:
        #Px = -cos(joint1)*(joint3_wrist1_x_offset*sin(joint2 + joint3) + joint2_joint3_x_offset*sin(joint2) + wrist1_cam_x_offset*sin(joint2 + joint3 + wrist1))             
        #Py = -sin(joint1)*(joint3_wrist1_x_offset*sin(joint2 + joint3) + joint2_joint3_x_offset*sin(joint2) + wrist1_cam_x_offset*sin(joint2 + joint3 + wrist1))             
        #Pz = joint1_joint2_z_offset + joint3_wrist1_x_offset*cos(joint2 + joint3) + joint2_joint3_x_offset*cos(joint2) + wrist1_cam_x_offset*cos(joint2 + joint3 + wrist1)
        
        #Px = (Px/-cos(joint1) - wrist1_cam_x_offset*sin(joint2 + joint3 + wrist1))*-cos(joint1)
        #Py = (Py/-sin(joint1) - wrist1_cam_x_offset*sin(joint2 + joint3 + wrist1))*-sin(joint1)
        #Pz = Pz - wrist1_cam_x_offset*cos(joint2 + joint3 + wrist1)
        #Px^2 + Py^2 + (Pz-joint1_joint2_z_offset)^2 = joint2_joint3_x_offset^2 + 2*cos(joint3)*joint2_joint3_x_offset*joint3_wrist1_x_offset + joint3_wrist1_x_offset^2
        #Px^2 + Py^2 = (joint3_wrist1_x_offset*sin(joint2 + joint3) + joint2_joint3_x_offset*sin(joint2))^2
        T_body_cam = np.eye(4, dtype=np.float32)
        T_body_cam[0,3] = Px_b
        T_body_cam[1,3] = Py_b
        T_body_cam[2,3] = Pz_b
        
        T_joint1_wrist1 = np.dot(T_joint1_body, T_body_cam)
        
        Px = T_joint1_wrist1[0,3]
        Py = T_joint1_wrist1[1,3]
        Pz = T_joint1_wrist1[2,3]

        joint1 = np.arctan2(-Py, -Px)
        
        Px = (Px/(-np.cos(joint1)) - wrist1_cam_x_offset*np.sin(cam_angle))*(-np.cos(joint1))
        Py = (Py/(-np.sin(joint1)) - wrist1_cam_x_offset*np.sin(cam_angle))*(-np.sin(joint1))
        Pz = Pz - wrist1_cam_x_offset*np.cos(cam_angle)
        
        cos_joint3 = (Px**2 + Py**2 + (Pz-joint1_joint2_z_offset)**2 - joint2_joint3_x_offset**2 - joint3_wrist1_x_offset**2)/(2*joint2_joint3_x_offset*joint3_wrist1_x_offset)
        if cos_joint3>1:
            cos_joint3 = 1
        if cos_joint3<-1:
            cos_joint3 = -1       
        sin_joint3 = np.sqrt(1-cos_joint3**2)
        joint3 = np.arctan2(sin_joint3, cos_joint3)
        
        a = joint3_wrist1_x_offset * sin_joint3
        c = joint3_wrist1_x_offset * cos_joint3 + joint2_joint3_x_offset
        d = np.sqrt(Px**2 + Py**2)
        e = joint3_wrist1_x_offset * cos_joint3 + joint2_joint3_x_offset
        f = -joint3_wrist1_x_offset * sin_joint3
        g = Pz - joint1_joint2_z_offset
        
        if a*f-c*e > 0:
            joint2 = np.arctan2(a*g-d*e, d*f-c*g)
        else:
            joint2 = np.arctan2(d*e-a*g, c*g-d*f)

        wrist1 = cam_angle - joint2 - joint3

        return [joint1, joint2, joint3, wrist1]
        
