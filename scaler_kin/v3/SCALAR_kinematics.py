from scaler_kin.v3.SCALER_v2_Leg_6DOF_gripper import Leg
import numpy as np
from scaler_kin import util




class scalar_k(object):
    def __init__(self):
        self.k_model = Leg()

    def scalar_forward_kinematics(self, which_leg, joint_angles, with_body=False, with_gripper=False, L_actuator=None, theta_actuator=None, body_angle = 0.0):
        #inputs:
        #which_leg -> leg index 0-3
        #joint angles -> [shoulder angle, q11, q21, wrist1, wrist2, wrist3]
        #with_body -> if True result is in body frame else in shoulder frame
        #with_gripper -> if True consider gripper as the end effector

        #output:
        #matrix T of wrist3 in shoulder frame if with_gripper is False else a list of two T matrices of toe1 and toe2

        if with_gripper==False:

            T_0_wrist3 = self.k_model.leg_fk_direct_calculation(body_angle, joint_angles, which_leg, which_link=9, use_quaternion = False)
            res = T_0_wrist3


            if with_body==False:
                T_0_shi = self.k_model.leg_fk_direct_calculation(body_angle, joint_angles, which_leg, which_link=1, use_quaternion = False)
                T_shi_0 = np.linalg.inv(T_0_shi)
                T_shi_wrist3 = np.dot(T_shi_0, T_0_wrist3)
                res = T_shi_wrist3

        else:
            if L_actuator is None or theta_actuator is None:
                print("You must input L_actuator and theta_actuator if with gripper!!!")
                return 0

            gripper_center_origin_toe_M, gripper_center_origin_toe_N = self.k_model.leg_gripper_fk(body_angle, joint_angles, L_actuator, theta_actuator, which_leg)

            T_toe1 = np.eye(4, dtype = np.float32)
            T_toe1[0:3,3] = gripper_center_origin_toe_M[0:3].reshape(-1)
            T_toe1[0:3,0:3] = util.quaternion_2_rotation_matrix(gripper_center_origin_toe_M[3:].reshape(4, ))

            T_toe2 = np.eye(4, dtype = np.float32)
            T_toe2[0:3,3] = gripper_center_origin_toe_N[0:3].reshape(-1)
            T_toe2[0:3,0:3] = util.quaternion_2_rotation_matrix(gripper_center_origin_toe_N[3:].reshape(4, ))

            if with_body==False:
                T_0_shi = self.k_model.leg_fk_direct_calculation(body_angle, joint_angles, which_leg, which_link=1, use_quaternion = False)
                T_shi_0 = np.linalg.inv(T_0_shi)
                T_toe1 = np.dot(T_shi_0, T_toe1)
                T_toe2 = np.dot(T_shi_0, T_toe2)

            res = [T_toe1, T_toe2]
        
            

        return res


    def scalar_inverse_kinematics(self, which_leg, target_T, is_first_ik=True, prev_angles=None, with_body=False, with_gripper=False, body_angle = 0.0):
        #inputs:
        #which_leg -> leg index 0-3
        #target_T -> if with_gripper is False, matrix T of wrist3 in shoulder frame else a list of two T matrices of toe1 and toe2
        #is_first_ik : if you don't have previous angles, set it to be True, else set it to be False
        #              if there's no previous angles, there would be some restriction for the situation: the initial position of the end effector can not be folded back to the shoulder, q11 -90 to 90 degrees and q21 0 to 180 degrees
        #prev_angles -> [shoulder angle, q11, q21, wrist1, wrist2, wrist3]
        #with_body -> if True the input matrix(matrice) is in body frame else in shoulder frame
        #with_gripper -> if True consider gripper as the end effector



        #output:
        #joint angles -> [shoulder angle, q11, q21, wrist1, wrist2, wrist3] if with_gripper is False else [[shoulder angle, q11, q21, wrist1, wrist2, wrist3], L_actuator, theta_actuator]


        new_prev_angles = np.array([0,0,np.pi/2,0,0,0,  0,0,np.pi/2,0,0,0,  0,0,np.pi/2,0,0,0,  0,0,np.pi/2,0,0,0])

        if not prev_angles is None:
            new_prev_angles = np.zeros(24, dtype=np.float32)
            new_prev_angles[which_leg*6:which_leg*6+6] = prev_angles

        if with_gripper==False:

            T_shi_wrist3 = target_T

            if with_body==True:
                T_0_shi = self.k_model.leg_fk_direct_calculation(body_angle, [0,0,np.pi/2,0,0,0], which_leg, which_link=1, use_quaternion = False)
                T_shi_0 = np.linalg.inv(T_0_shi)
                T_shi_wrist3 = np.dot(T_shi_0, T_shi_wrist3)

            quaternion_state = util.rotation_matrix_2_quaternion(T_shi_wrist3[0:3, 0:3])

            shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi = self.k_model.leg_ik_direct_calculation_6DoF(T_shi_wrist3[0:3,3].tolist(), quaternion_state, which_leg, is_first_ik = is_first_ik, prev_angles = new_prev_angles)

            return [shoulder_angle, q11, q21, qw1, qw2, qw3]

        else:

            T_toe1 = target_T[0]
            T_toe2 = target_T[1]

            if with_body==False:
                T_0_shi = self.k_model.leg_fk_direct_calculation(body_angle, [0,0,np.pi/2,0,0,0], which_leg, which_link=1, use_quaternion = False)
                T_toe1 = np.dot(T_0_shi, T_toe1)
                T_toe2 = np.dot(T_0_shi, T_toe2)

            quaternion_state_1 = util.rotation_matrix_2_quaternion(T_toe1[0:3, 0:3])
            T_toe1 = T_toe1[0:3,3].tolist() + quaternion_state_1.tolist()
            quaternion_state_2 = util.rotation_matrix_2_quaternion(T_toe2[0:3, 0:3])
            T_toe2 = T_toe2[0:3,3].tolist() + quaternion_state_2.tolist()

            return self.k_model.leg_gripper_ik(np.array(T_toe1), np.array(T_toe2), body_angle, which_leg, is_first_ik, new_prev_angles)


    def scalar_forward_kinematics_3DoF(self, which_leg, joint_angles, with_body=False, body_angle = 0.0, output_xyz = False):
        #inputs:
        #which_leg -> leg index 0-3
        #joint angles -> [shoulder angle, q11, q21]
        #with_body -> if True result is in body frame else in shoulder frame

        #output:
        #matrix T of wrist3 in body/shoulder frame if output_xyz is false else just the xyz postion

        new_joint_angles = np.zeros(6, dtype=np.float32)
        new_joint_angles[0:3]=joint_angles

        T_0_wrist3 = self.k_model.leg_fk_direct_calculation(body_angle, new_joint_angles, which_leg, which_link=11, use_quaternion = False)
        res = T_0_wrist3


        if with_body==False:
            T_0_shi = self.k_model.leg_fk_direct_calculation(body_angle, new_joint_angles, which_leg, which_link=1, use_quaternion = False)
            T_shi_0 = np.linalg.inv(T_0_shi)
            T_shi_wrist3 = np.dot(T_shi_0, T_0_wrist3)
            res = T_shi_wrist3
        
        if output_xyz == False:
            return res
        else:
            return res[0:3,3].reshape(-1)


    def scalar_inverse_kinematics_3DoF(self, which_leg, target_pose, is_first_ik=True, prev_angles=None, with_body=False, body_angle = 0.0, input_xyz=False):
        #inputs:
        #which_leg -> leg index 0-3
        #target_xyz -> target xyz position of toe in body/shoulder frame if input_xyz is true else the T matrix
        #is_first_ik : if you don't have previous angles, set it to be True, else set it to be False
        #              if there's no previous angles, there would be some restriction for the situation: the initial position of the end effector can not be folded back to the shoulder, q11 -90 to 90 degrees and q21 0 to 180 degrees
        #prev_angles -> [shoulder angle, q11, q21]
        #with_body -> if True the input matrix is in body frame else in shoulder frame



        #output:
        #joint angles -> [shoulder angle, q11, q21]

        if input_xyz is True:
            T_shi_wrist3 = np.eye(4, dtype=np.float32)
            T_shi_wrist3[0:3,3] = np.array(target_pose).reshape(-1)
        else:
            T_shi_wrist3 = target_pose

        if with_body==True:
            T_0_shi = self.k_model.leg_fk_direct_calculation(body_angle, [0,0,np.pi/2,0,0,0], which_leg, which_link=1, use_quaternion = False)
            T_shi_0 = np.linalg.inv(T_0_shi)
            T_shi_wrist3 = np.dot(T_shi_0, T_shi_wrist3)

        new_prev_angles = np.array([0,0,np.pi/2,0,0,0,  0,0,np.pi/2,0,0,0,  0,0,np.pi/2,0,0,0,  0,0,np.pi/2,0,0,0])

        if not prev_angles is None:
            new_prev_angles = np.zeros(24, dtype=np.float32)
            new_prev_angles[which_leg*6:which_leg*6+3] = prev_angles

        sol = self.k_model.leg_ik_direct_calculation_3DoF(T_shi_wrist3[0:3,3].tolist(), which_leg, is_first_ik = is_first_ik, prev_angles = new_prev_angles)

        return sol

    def scalar_forward_kinematics_4DoF(self, which_leg, joint_angles, with_body=False, body_angle=0.0,
                                       output_xyzq=False):
        # First get 3DoF transformation matrix
        T_3DoF = self.scalar_forward_kinematics_3DoF(which_leg, joint_angles[:3], with_body, body_angle)

        # Create the 4th joint rotation matrix around x-axis
        theta = joint_angles[3]  # 4th joint angle
        T_4th_joint = np.array([
            [1, 0, 0, 0],
            [0,np.cos(theta), -np.sin(theta), 0],
            [0,np.sin(theta), np.cos(theta), 0],
            [0, 0, 0, 1]
        ])

        # Calculate final transformation matrix
        T_final = np.dot(T_3DoF, T_4th_joint)

        if output_xyzq:
            return np.append(T_final[:3, 3], util.rotation_2_euler(T_final[:3, :3])[0] )# only return the rotation around x
        else:
            return T_final

    def scaler_inverse_kinematics_4DoF(self, which_leg, target_pose, is_first_ik=True, prev_angles=None,
                                       with_body=False, body_angle=0.0, input_xyzq=True):
        if input_xyzq:
            # Separate desired y-axis rotation from target_pose
            target_xyz, desired_x_rotation = target_pose[:3], target_pose[3]
        else:
            # Extract desired y-axis rotation from transformation matrix
            target_xyz = target_pose[:3, 3]
            desired_x_rotation = util.rotation_2_euler(target_pose[0:3,0:3])[0]



        # Calculate 3DoF inverse kinematics
        ik_3DoF = self.scalar_inverse_kinematics_3DoF(which_leg, target_xyz, is_first_ik,
                                                      prev_angles[:3] if prev_angles is not None else None,
                                                         with_body, body_angle, input_xyz=True)

        # Calculate the current y-axis rotation from 3DoF IK solutions
        T_3DoF = self.scalar_forward_kinematics_3DoF(which_leg, ik_3DoF, with_body, body_angle)
        current_x_rotation = util.rotation_2_euler(T_3DoF[0:3,0:3])[0]

        # Calculate the additional rotation needed at the 4th joint
        theta_4th_joint = desired_x_rotation - current_x_rotation

        # Combine results
        return np.append(ik_3DoF, theta_4th_joint)

    @staticmethod
    def set_previous_angles(which_leg, prev_angles, total_legs=4, angles_per_leg=6):
        """
        Sets the previous angles for a given leg in a zero-initialized array.

        Parameters:
        - which_leg: The index of the leg for which to set the previous angles (0-based index).
        - prev_angles: List or array of previous angles for the leg.
        - total_legs: Total number of legs in the robot (default is 4).
        - angles_per_leg: Number of angles per leg (default is 6).

        Returns:
        - new_prev_angles: A numpy array containing the previous angles for all legs,
          with non-zero values for the specified leg.
        """
        new_prev_angles = np.zeros(total_legs * angles_per_leg, dtype=np.float32)
        start_idx = which_leg * angles_per_leg
        end_idx = start_idx + len(prev_angles)
        new_prev_angles[start_idx:end_idx] = prev_angles
        return new_prev_angles

















