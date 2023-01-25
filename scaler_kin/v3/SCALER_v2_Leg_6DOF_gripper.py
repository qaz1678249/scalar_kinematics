import scaler_kin.SCALER_v2_Leg_6DOF_gripper as old_SCALER_v2_Leg_6DOF_gripper

from scaler_kin import utils_SCALER as Scaler_utils
from scaler_kin import util
from scaler_kin.wrap_to_pi import wrap_to_pi
from scipy.optimize import fsolve

import numpy as np

from .hardware_constants import consts

robot_name = 'SCALER_climbing_6DoF_gripper'
robot_consts = consts[robot_name]

scaler_std_utils = Scaler_utils.ScalerStandardUtilMethods

# The Z Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
TOP_LEG_SERVO_OFF_Z = robot_consts.TOP_LEG_SERVO_OFF_Z

BOTTOM_LEG_SERVO_OFF_Z = robot_consts.BOTTOM_LEG_SERVO_OFF_Z

# Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the Position (X,Z) of the
# Parallel Leg Origin with respect to Shoulder's Reference Frame) [units: mm]
LEG_ORIGIN_X = robot_consts.LEG_ORIGIN_X
LEG_ORIGIN_Z = robot_consts.LEG_ORIGIN_Z

# Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset Positions of the Top and Bottom Leg
# Servos). [units: radians]
LEG_ROT_OFF_ANGLE = robot_consts.LEG_ROT_OFF_ANGLE

# Length of the Leg Link 1 (Links from the Top/Bottom Leg Servos to the Elbow Joints). [units: mm]
L_LEG_LINK_1 = robot_consts.L_LEG_LINK_1

# Length of the Leg Link 2 (Links from the Elbow Joints to the Wrist Servo). [units: mm]
L_LEG_LINK_2 = robot_consts.L_LEG_LINK_2

L_LEG_LINK_2_NEW = robot_consts.L_LEG_LINK_2_NEW

# Length of the Wrist Link [units: mm]
L_WRIST = robot_consts.L_WRIST

# Length of the Gripper offset (x-component) from the Spherical Joint [units: mm]
L_GRIPPER_OFFSET_X = robot_consts.L_GRIPPER_OFFSET_X

# Length of the Gripper offset (y-component) from the Spherical Joint [units: mm]
L_GRIPPER_OFFSET_Y = robot_consts.L_GRIPPER_OFFSET_Y

# Length Between the Leg Servos (Servo Pair) [units: mm]
L_BLSP = robot_consts.L_BLSP

# Length of the Battery Link [units: mm]
L_BATTERY = robot_consts.L_BATTERY

# Length of the Body Link [units: mm]
L_BL = robot_consts.L_BL


L_LEG_LINK_A23_WRIST = robot_consts.L_LEG_LINK_A23_WRIST

L_LEG_LINK_A22_WRIST = robot_consts.L_LEG_LINK_A22_WRIST

LEG_GAMMA_OFF_ANGLE = robot_consts.LEG_GAMMA_OFF_ANGLE

LEG_THETA_1_OFF_ANGLE = robot_consts.LEG_THETA_1_OFF_ANGLE


T_wrist_gripper_0and2 = robot_consts.T_wrist_gripper_0and2

T_wrist_gripper_1and3 = robot_consts.T_wrist_gripper_1and3

T_wrist_gripper_0and2_inv = robot_consts.T_wrist_gripper_0and2_inv

T_wrist_gripper_1and3_inv = robot_consts.T_wrist_gripper_1and3_inv

# State Dimension of each Footstep.
DIM_FOOTSTEP = robot_consts.DIM_FOOTSTEP

# gripper hardware constants
IJ_X = robot_consts.L_GRIPPER_2D_L_IJ_X
IJ_Y = robot_consts.L_GRIPPER_2D_L_IJ_Y

L1 = robot_consts.L_GRIPPER_2D_L1
L2 = robot_consts.L_GRIPPER_2D_L2
L3 = robot_consts.L_GRIPPER_2D_L3
L4 = robot_consts.L_GRIPPER_2D_L4
L5 = robot_consts.L_GRIPPER_2D_L5
L6 = robot_consts.L_GRIPPER_2D_L6
L7 = robot_consts.L_GRIPPER_2D_L7
L8 = robot_consts.L_GRIPPER_2D_L8
L9 = robot_consts.L_GRIPPER_2D_L9
L10 = robot_consts.L_GRIPPER_2D_L10
L11 = robot_consts.L_GRIPPER_2D_L11
L12 = robot_consts.L_GRIPPER_2D_L12
L13 = robot_consts.L_GRIPPER_2D_L13
L14 = robot_consts.L_GRIPPER_2D_L14
L15 = robot_consts.L_GRIPPER_2D_L15
scaler_helper = scaler_std_utils()



class Leg(old_SCALER_v2_Leg_6DOF_gripper.Leg):
    @staticmethod
    def leg_fk_direct_calculation(body_angle, theta_angles, which_leg, which_link=-1, use_quaternion = True):
        """ Calculate the Full Forward Kinematics of the Parallel Legs
        This method calculates the full forward kinematics of the parallel legs for SCALER_v2 (6-DoF Climbing
        Configuration w/o Gripper Kinematics). This assumes that the full forward kinematics specifies the position of
        the center of the gripper and the orientation of the gripper (quaternion) with respect to SCALER_v2's Body
        Reference Frame.
        Args:
            body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2 (6-DoF Climbing
                        Configuration w/o Gripper Kinematics)) [units: radians]
            theta_angles: This is a [1 x 6] Matrix which contains the direct joint angles of a single SCALER_v2 leg. It
                          is important to note that these angles are directly measured from the leg reference frame.
                          More specifically, they are the active joint angles (NOT in the motor frame)
                theta_angles = [shoulder_angle, q11, q21]
                    shoulder_angle  = Angle of the Shoulder Joint (i.e., MOTOR_ID = 1, 4, 7, 10)
                                      [units: radians]
                    q11             = Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 2, 5, 8, 11)
                                      [units: radians]
                    q21             = Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 3, 6, 9, 12)
                                      [units: radians]
                    qw1             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 13, 16, 19, 22)
                                      [units: radians]
                    qw2             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 14, 17, 20, 23)
                                      [units: radians]
                    qw3             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 15, 18, 21, 24)
                                      [units: radians]
            which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).


            which_link: This is an index variable that specifies which link we want to calculate the forward kinematics. All the frames are from body frame
                which_link = 1: shoulder frame before rotation of shoulder angle
                which_link = 2: frame A after rotation of q11
                which_link = 3: frame F after rotation of q21
                which_link = 4: frame B after rotation of q12
                which_link = 5: frame E after rotation of q22
                which_link = 6: frame C after rotation of q13
                which_link = 7: frame wrist1 after rotation of qw1
                which_link = 8: frame wrist2 after rotation of qw2
                which_link = 9: frame wrist3 after rotation of qw3
                which_link = 11: frame wrist1 without rotation from frame E
                (by default) all the other which_link: frame gripper_center

            use_quaternion: (by default)If it is True, the final result of frame gripper_center would be in the format of [posx,posy,posz, qw, qx, qy, qz], but all the other returned links would be T matrix
                            If it is False, all the outputs would be T matrix
                            

                


        Returns:
            state: The State of the specified leg for SCALER_v2. This is a [7 x 1] Matrix that contains the XYZ
                   Position of a single leg's footstep (center of gripper) and the Orientation (quaternion) of the
                   gripper with respect to the body frame.
                state[0]: X Position of the footstep with respect to the body frame. [units: mm]
                state[1]: Y Position of the footstep with respect to the body frame. [units: mm]
                state[2]: Z Position of the footstep with respect to the body frame. [units: mm]
                state[3]: W (Scalar) Orientation of the gripper (quaternion) with respect to the body frame.
                state[4]: X Orientation of the gripper (quaternion) with respect to the body frame.
                state[5]: Y Orientation of the gripper (quaternion) with respect to the body frame.
                state[6]: Z Orientation of the gripper (quaternion) with respect to the body frame.

            or T matrix(depands on use_quaternion)

        """
        # Unpack the theta angles
        shoulder_angle = theta_angles[0]
        q11 = theta_angles[1]
        q21 = theta_angles[2]
        th4 = theta_angles[3]
        th5 = theta_angles[4]
        th6 = theta_angles[5]

        # TODO: Eventually, the State Dimension should be state = np.zeros([DIM_FOOTSTEP, 1])
        state = np.zeros([3, 1])

        shoulder_vertex = scaler_std_utils.find_shoulder_vertices(body_angle,
                                                                  use_find_specific_shoulder=True,
                                                                  which_leg=which_leg)

        
        if which_leg==0 or which_leg==3:
            T_0_shi = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0, 0],
                                [np.sin(np.pi/2),  np.cos(np.pi/2), 0, 0],
                                [            0,              0, 1, 0],
                                [0,0,0,1]])
        else:
            T_0_shi = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2), 0, 0],
                                [np.sin(-np.pi/2),  np.cos(-np.pi/2), 0, 0],
                                [            0,              0, 1, 0],
                                [0,0,0,1]])
        T_0_shi[0:3,3] = np.array(shoulder_vertex)       
        if which_link==1:
            return T_0_shi


        T_shi_A_rot = np.array([[np.cos(shoulder_angle), -np.sin(shoulder_angle), 0, 0],
                               [np.sin(shoulder_angle), np.cos(shoulder_angle),  0, 0],
                               [                     0,                       0, 1, 0],
                               [0,0,0,1]])

        T_shi_A_t = np.array([[1, 0, 0, LEG_ORIGIN_X],
                              [0, 1, 0, 0],
                              [0, 0, 1, TOP_LEG_SERVO_OFF_Z],
                              [0,0,0,1]])

        T_shi_A = np.dot(T_shi_A_rot, T_shi_A_t)

        #if which_link==100:
        #    return np.dot(T_0_shi, T_shi_A_rot)


        T_shi_A_cord_rot_1 = np.array([[1,                 0,                 0, 0],
                                       [0,   np.cos(np.pi/2),  -np.sin(np.pi/2), 0],
                                       [0,   np.sin(np.pi/2),   np.cos(np.pi/2), 0],
                                       [0,0,0,1]])

        T_shi_A_cord_rot_2 = np.array([[np.cos(-np.pi/2),  -np.sin(-np.pi/2),  0, 0],
                                       [np.sin(-np.pi/2),   np.cos(-np.pi/2),  0, 0],
                                       [               0,                  0,  1, 0],
                                       [0,0,0,1]])
        T_shi_A_cord_rot = np.dot(T_shi_A_cord_rot_1, T_shi_A_cord_rot_2)



        T_0_A = np.dot(np.dot(T_0_shi, T_shi_A), T_shi_A_cord_rot)

        if which_link==2:
            T_A_rot = np.array([[np.cos(q11),  -np.sin(q11),  0, 0],
                                [np.sin(q11),   np.cos(q11),  0, 0],
                                [               0,                  0,  1, 0],
                                [0,0,0,1]])
            return np.dot(T_0_A, T_A_rot)

        T_0_F = np.array(T_0_A)
        T_0_F[2,3] = T_0_F[2,3] - (TOP_LEG_SERVO_OFF_Z - BOTTOM_LEG_SERVO_OFF_Z)

        if which_link==3:
            T_F_rot = np.array([[np.cos(q21),  -np.sin(q21),  0, 0],
                                [np.sin(q21),   np.cos(q21),  0, 0],
                                [               0,                  0,  1, 0],
                                [0,0,0,1]])
            return np.dot(T_0_F, T_F_rot)

        
        #leg_origin_vertex = T_0_A[0:3,3]


        fk_variable_0, fk_variable_1, fk_variable_2, fk_variable_3, L_EB, GEC_ANGLE = Leg.find_fk_variables(q11, q21)

        if which_link==4 or which_link==6:
            COS_BCE = (- L_EB**2 + L_LEG_LINK_2_NEW**2 + L_LEG_LINK_2**2) / (2*L_LEG_LINK_2*L_LEG_LINK_2_NEW)
            SIN_BCE = np.sqrt(1 - COS_BCE**2)
            BCE_ANGLE = np.arctan2(SIN_BCE, COS_BCE)
            
            HCE_ANGLE = np.pi/2 - GEC_ANGLE
            BCH_ANGLE = BCE_ANGLE - HCE_ANGLE

            T_A_B = np.array([[np.cos(BCH_ANGLE), -np.sin(BCH_ANGLE),0,L_LEG_LINK_1 * np.cos(q11)],
                              [np.sin(BCH_ANGLE),  np.cos(BCH_ANGLE),0,L_LEG_LINK_1 * np.sin(q11)],
                              [0,0,1,0],
                              [0,0,0,1]])
            T_0_B = np.dot(T_0_A, T_A_B)

            if which_link == 4:
                return T_0_B
            else:
                DCJ_ANGLE = np.pi - BCE_ANGLE - (np.pi/2 - LEG_THETA_1_OFF_ANGLE)
                T_B_C = np.array([[np.cos(DCJ_ANGLE), -np.sin(DCJ_ANGLE),0,L_LEG_LINK_2],
                                  [np.sin(DCJ_ANGLE),  np.cos(DCJ_ANGLE),0,0],
                                  [0,0,1,0],
                                  [0,0,0,1]])
                T_0_C = np.dot(T_0_B, T_B_C)
                return T_0_C


        GET_ANGLE = GEC_ANGLE + LEG_THETA_1_OFF_ANGLE
        GET_ANGLE_OFF = np.pi/2 - GET_ANGLE
        T_A_E = np.array([[np.cos(-GET_ANGLE_OFF), -np.sin(-GET_ANGLE_OFF),0,L_LEG_LINK_1 * np.cos(q21)+L_BLSP],
                          [np.sin(-GET_ANGLE_OFF),  np.cos(-GET_ANGLE_OFF),0,L_LEG_LINK_1 * np.sin(q21)],
                          [0,0,1,0],
                          [0,0,0,1]])
        T_0_E = np.dot(T_0_A, T_A_E)
        if which_link==5:
            return T_0_E


        #T_A_TOE = np.array([[1,0,0,fk_variable_3],
        #                    [0,1,0,fk_variable_2],
        #                    [0,0,1,0],
        #                    [0,0,0,1]])

        #T_0_TOE = np.dot(T_0_A, T_A_TOE)

        #state[0] = leg_origin_vertex[0] + np.cos(shoulder_angle) * fk_variable_2

        #state[1] = leg_origin_vertex[1] + np.sin(shoulder_angle) * fk_variable_2

        #state[2] = leg_origin_vertex[2] - fk_variable_3

        T_E_wrist = np.array([[1,0,0,L_LEG_LINK_A22_WRIST],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])
        T_0_wrist = np.dot(T_0_E, T_E_wrist)


        if which_link==11:
            return T_0_wrist


        T_wrist_1 = np.array([ [ -np.cos(th4),   np.sin(th4),      0,         0],
                               [  np.sin(th4),   np.cos(th4),      0,         0],
                               [            0,             0,     -1,         0],
                               [            0,             0,      0,         1] ])

        T_0_wrist_1 = np.dot(T_0_wrist,T_wrist_1)

        if which_link==7:
            return T_0_wrist_1


        T_wrist_2 = np.array([ [ -np.sin(th5),  -np.cos(th5),      0,         0],
                               [            0,             0,      1,         0],
                               [ -np.cos(th5),   np.sin(th5),      0,         0],
                               [            0,             0,      0,         1] ])

        T_0_wrist_2 = np.dot(T_0_wrist_1,T_wrist_2)

        if which_link==8:
            return T_0_wrist_2

        T_wrist_3 =  np.array([ [ -np.sin(th6),  -np.cos(th6),      0,         0],
                                [            0,             0,      1,         0],
                                [ -np.cos(th6),   np.sin(th6),      0,         0],
                                [            0,             0,      0,         1] ])
        T_0_wrist_3 = np.dot(T_0_wrist_2,T_wrist_3)

        if which_link==9:
            return T_0_wrist_3

        if which_leg == 0 or which_leg == 2:
            T_0_gripper_center =  np.dot(T_0_wrist_3, T_wrist_gripper_0and2)
        else:
            T_0_gripper_center =  np.dot(T_0_wrist_3, T_wrist_gripper_1and3)

        if use_quaternion == True:
            state = np.zeros([7, 1])
            quaternion_state = util.rotation_matrix_2_quaternion(T_0_gripper_center[0:3, 0:3])
            state[0:3,0] = T_0_gripper_center[0:3,3]
            state[3:7,0] = quaternion_state
            return state
        else:
            return T_0_gripper_center
            
    @staticmethod
    def leg_gripper_fk(body_angle, theta_angles, L_actuator, theta_actuator, which_leg):
        """ Calculate the Full Forward Kinematics of the Parallel Legs
                This method calculates the full forward kinematics of the parallel legs for SCALER_v2 (6-DoF Climbing
                Configuration with Gripper Kinematics). This assumes that the full forward kinematics specifies the position of
                the gripper fingertips and the orientation of the fingertip orientation (euler) with respect to SCALER_v2's Body
                Reference Frame.
                Args:
                    body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2 (6-DoF Climbing
                                Configuration w/o Gripper Kinematics)) [units: radians]
                    theta_angles: This is a [1 x 3] Matrix which contains the direct joint angles of a single SCALER_v2 leg. It
                                  is important to note that these angles are directly measured from the leg reference frame.
                                  More specifically, they are the active joint angles (NOT in the motor frame)
                        theta_angles = [shoulder_angle, q11, q21]
                            shoulder_angle  = Angle of the Shoulder Joint (i.e., MOTOR_ID = 1, 4, 7, 10)
                                              [units: radians]
                            q11             = Angle of the Top Leg Servo Joint (i.e., MOTOR_ID = 2, 5, 8, 11)
                                              [units: radians]
                            q21             = Angle of the Bottom Leg Servo Joint (i.e., MOTOR_ID = 3, 6, 9, 12)
                                              [units: radians]
                            qw1             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 13, 16, 19, 22)
                                              [units: radians]
                            qw2             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 14, 17, 20, 23)
                                              [units: radians]
                            qw3             = Angle of the First Wrist Servo Joint (i.e., MOTOR_ID = 15, 18, 21, 24)
                                              [units: radians]
                    L_actuator: This is the length of the linear actuator of the gripper (i.e., MOTOR_ID = ....), scalar
                    value [units: mm]
                    theta_actuator: This is the offset angle specified by the hinge on the gripper, scalar value
                    [units: radians]
                    which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                        For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                        which_leg = 0: Leg 1 (Front Right Leg).
                        which_leg = 1: Leg 2 (Back Right Leg).
                        which_leg = 2: Leg 3 (Back Left Leg).
                        which_leg = 3: Leg 4 (Front Left Leg).
                Returns:
                    gripper_center_body_toe_M: Finger 1 vector in 3D-space (X, Y, Z, w, x, y, z) from the Body to
                    the desired Finger Tip Positions and quaternion Rotation (Finger 1 or Finger M). [dim: 7 x 1] [units: mm]
                """
        # We calculate the FK of the 6 DoF leg (outputs the position and quaterninon orientation of the center of gripper)
        state_gripper_center = Leg.leg_fk_direct_calculation(body_angle,theta_angles,which_leg)

        # We get the center of gripper position
        x_pos,y_pos,z_pos = state_gripper_center[0], state_gripper_center[1], state_gripper_center[2]
        # We convert quaterninon orientation to 3x3 rotation matrix
        w,x_rot,y_rot,z_rot = state_gripper_center[3], state_gripper_center[4], state_gripper_center[5], state_gripper_center[6]
        R_BC = util.quaternion_2_rotation_matrix(np.array([w, x_rot, y_rot, z_rot]))
        # Create T matrix that specifies the frame from center of gripper to body
        T_BC = np.array([[R_BC[0,0],R_BC[0,1],R_BC[0,2],x_pos],
                        [R_BC[1,0],R_BC[1,1],R_BC[1,2],y_pos],
                        [R_BC[2,0],R_BC[2,1],R_BC[2,2],z_pos],
                        [0,0,0,1]], dtype=np.float32)

        # We get the T matrix for fingertip position and orientation of fingertip 1 and fingertip 2 in
        # center of gripper frame
        gripper_center_origin_toe_M, gripper_center_origin_toe_N = \
            scaler_std_utils.gripper_2d_two_finger_fk_noOffset(T_BC,L_actuator,theta_actuator,which_leg)
        # TODO: eventually we should switch which local gripper FK IK be used in 7 DoF FK IK
        # gripper_center_origin_toe_M, gripper_center_origin_toe_N = \
        #     scaler_std_utils.gripper_2d_two_finger_fk(T_BC,L_actuator,theta_actuator,which_leg)

        return gripper_center_origin_toe_M, gripper_center_origin_toe_N
        
        
    @staticmethod
    def leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion, which_leg=-1, is_first_ik= True, prev_angles = None):
        """ Calculates the Inverse Kinematics of the Parallel Leg and Wrist.
        This method calculates the inverse kinematics of the parallel leg for the SCALER_v2 given the position of
        the toe and the desired orientation of the spherical wrist (quaternion).

        verified joint angle range:
        shoulder_angle -> [-np.pi/3,np.pi/3]
                   q11 -> (np.pi*1.5,np.pi*2)
                   q21 -> [np.pi/4,np.pi/4*3]
                   qw1 -> [-np.pi/4,np.pi/4]
                   qw2 -> [-np.pi/4,np.pi/4]
                   qw3 -> [-np.pi/4,np.pi/4]

        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                              Body Frame of SCALER_v2. [dim: 1 x 4]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            q11             : Angle of the Top Leg Servo Joint [units: radians]
            q12             : Angle of the Top Leg Elbow Joint [units: radians] (would be always 0)
            q13             : Angle to satisfy five-bar closed chain (q13 = q21 + q22 - q11 - q12) [units: radians] (would be always 0)
            q21             : Angle of the Bottom Leg Servo Joint [units: radians]
            q22             : Angle of the Bottom Leg Elbow Joint [units: radians] (would be always 0)
            qw1             : Angle of the First Wrist Servo Joint [units: radians]
            qw2             : Angle of the Second Wrist Servo Joint [units: radians]
            qw3             : Angle of the Third Wrist Servo Joint [units: radians]
            phi             : Angle of Orientation Constraint of Parallel Leg Mechanism
                              (i.e., phi = q21 + q22 = q11 + q12 + q13) [units: radians] (would be always 0)
        """


        # Rotation Matrix of the desired wrist orientation with respect to SCALER v2 Body Reference Frame.
        rot_gripper = util.quaternion_2_rotation_matrix(wrist_quaternion)
        T_shoulder_gripper = np.eye(4, dtype=np.float32)
        T_shoulder_gripper[0:3,0:3] = rot_gripper
        T_shoulder_gripper[0:3,3] = np.array(shoulder_2_toe_xyz).reshape(-1)
        if which_leg == 0 or which_leg == 2:
            T_shoulder_wrist3 = np.dot(T_shoulder_gripper, T_wrist_gripper_0and2_inv)
        elif which_leg == 1 or which_leg == 3:
            T_shoulder_wrist3 = np.dot(T_shoulder_gripper, T_wrist_gripper_1and3_inv)
        else:
            print("Invalid leg index")
            return

        P_shoulder_wrist = T_shoulder_wrist3[0:3,3].reshape(-1)

        First_3_joint = Leg.leg_ik_direct_calculation_3DoF(P_shoulder_wrist, which_leg, is_first_ik, prev_angles)

        shoulder_angle = First_3_joint[0]
        q11 = First_3_joint[1]
        q21 = First_3_joint[2]

        if q11 < -np.pi/2:
            q11 = -np.pi-q11
        if q11 > np.pi/2:
            q11 = np.pi-q11
        if q21 < 0:
            q21 = -q21



        T_0_shoulder = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 1, use_quaternion = False)
        T_0_wrist = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 11, use_quaternion = False)
        T_shoulder_wrist = np.dot(np.linalg.inv(T_0_shoulder), T_0_wrist)

        T_wrist_wrist3 = np.dot(np.linalg.inv(T_shoulder_wrist), T_shoulder_wrist3)
        

        rot_wrist = T_wrist_wrist3[0:3,0:3]

        # Rotation Matrix of the wrist orientation with respect to the Wrist Reference Frame (before 3-DoF wrist
        # rotations). This can be thought of as the rotation matrix of the wrist (so we determine what the 3-DoF
        # spherical joint angle need to be).

        r_w_11, r_w_12, r_w_13, \
            r_w_21, r_w_22, r_w_23, \
            r_w_31, r_w_32, r_w_33 = util.unpack_rotation_matrix(rot_wrist)

        # Trivial solution to the Spherical Joint
        qw2 = np.arctan2(-r_w_33, np.sqrt(r_w_13 ** 2 + r_w_23 ** 2))
        qw1 = np.arctan2(-r_w_23 / np.cos(qw2),  r_w_13 / np.cos(qw2))
        qw3 = np.arctan2(-r_w_31 / np.cos(qw2), -r_w_32 / np.cos(qw2))

        return [shoulder_angle, q11, 0, 0, q21, 0, qw1, qw2, qw3, 0]
        
        
    @staticmethod
    def leg_gripper_ik(gripper_center_body_toe_M, gripper_center_body_toe_N, body_angle, which_leg=-1, is_first_ik=True, prev_angles=None):
        """ Inverse Kinematics of the Two Finger 2D Gripper
                Args:
                    gripper_center_body_toe_M: Finger 1 vector in 3D-space (X, Y, Z, w, x, y, z) from the Body to the desired Finger Tip
                                             Positions and quaternion rotation (Finger 1 or Finger M). [dim: 7 x 1] [units: mm]
                        gripper_center_body_toe_M = [finger1_X, finger1_Y, finger1_Z, finger2_w, finger2_x, finger2_y, finger2_z]
                    gripper_center_body_toe_M: Finger 2 vector in 3D-space (X, Y, Z) from the Body to the desired Finger Tip
                                            Positions (Finger 2 or Finger N). [dim: 7 x 1] [units: mm]
                        gripper_center_body_toe_M = [finger2_X, finger2_Y, finger2_Z, finger2_w, finger2_x, finger2_y, finger2_z]
                    body_angle: This is the angle of the body posture motor (i.e., MOTOR_ID = 0 for SCALER_v2) [units: radians]
                    which_leg: This is an index variable that specifies which leg we want to calculate the forward kinematics.
                        For SCALER_v2, we have four legs in total. This means that which_leg has the following options
                        which_leg = 0: Leg 1 (Front Right Leg).
                        which_leg = 1: Leg 2 (Back Right Leg).
                        which_leg = 2: Leg 3 (Back Left Leg).
                        which_leg = 3: Leg 4 (Front Left Leg).
        """
        # We calculate the linear actuator and gripper offset angle value (i.e., IK of gripper only)
        # TODO: eventually we should switch which local gripper FK IK be used in 7 DoF FK IK
        L_actuator, theta_actuator, T_Gripper_Center_Body, _, _ = scaler_std_utils.gripper_2d_two_finger_ik_noOffset(gripper_center_body_toe_M,gripper_center_body_toe_N)


        #shoulder_vertex = Scaler_utils.ScalerStandardUtilMethods.find_shoulder_vertices(body_angle,
        #                                                                                use_find_specific_shoulder=True,
        #                                                                                which_leg=which_leg)

        #shoulder_2_toe_xyz = T_Gripper_Center_Body[0:3,3]-shoulder_vertex
        #wrist_quaternion = util.rotation_matrix_2_quaternion(T_Gripper_Center_Body[0:3,0:3])
        T_0_shi = Leg.leg_fk_direct_calculation(body_angle, [0,0,0,0,0,0], which_leg, which_link=1, use_quaternion = False)
        T_shi_0 = np.linalg.inv(T_0_shi)
        T_shi_gripper = np.dot(T_shi_0, T_Gripper_Center_Body)
        shoulder_2_toe_xyz = T_shi_gripper[0:3,3]
        wrist_quaternion = util.rotation_matrix_2_quaternion(T_shi_gripper[0:3, 0:3])
        

        if which_leg == -1:
            [shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi] = \
                Leg.leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion, is_first_ik=is_first_ik, prev_angles=prev_angles)
        else:
            [shoulder_angle, q11, q12, q13, q21, q22, qw1, qw2, qw3, phi] = \
                Leg.leg_ik_direct_calculation(shoulder_2_toe_xyz, wrist_quaternion, which_leg=which_leg, is_first_ik=is_first_ik, prev_angles=prev_angles)

        joint_angles = [shoulder_angle, q11, q21, qw1, qw2, qw3]


        return [joint_angles, L_actuator, theta_actuator]


    @staticmethod
    def leg_ik_direct_calculation_6DoF(shoulder_2_toe_xyz, wrist_quaternion, which_leg=-1, is_first_ik= True, prev_angles = None):
        """ Calculates the Inverse Kinematics of the Parallel Leg and Wrist.
        This method calculates the inverse kinematics of the parallel leg for the SCALER_v2 given the position of
        the toe and the desired orientation of the spherical wrist (quaternion).

        verified joint angle range:
        shoulder_angle -> [-np.pi/3,np.pi/3]
                   q11 -> (np.pi*1.5,np.pi*2)
                   q21 -> [np.pi/4,np.pi/4*3]
                   qw1 -> [-np.pi/4,np.pi/4]
                   qw2 -> [-np.pi/4,np.pi/4]
                   qw3 -> [-np.pi/4,np.pi/4]

        Args:
            shoulder_2_toe_xyz: Vector in 3D-space (X, Y, Z) from the Shoulder Joint to the desired Toe Position.
                                [dim: 3 x 1][units: mm]
            wrist_quaternion: Quaternion (w, x, y, z) that represents the desired wrist orientation with respect to the
                              Body Frame of SCALER_v2. [dim: 1 x 4]
            which_leg (optional arg): This is an index variable that specifies which leg we want to find the shoulder
                                      vertex for. [default value: -1].
                For SCALER_v1, we have four legs in total. This means that which_leg has the following options
                which_leg = 0: Leg 1 (Front Right Leg).
                which_leg = 1: Leg 2 (Back Right Leg).
                which_leg = 2: Leg 3 (Back Left Leg).
                which_leg = 3: Leg 4 (Front Left Leg).
        Returns:
            shoulder_angle  : Angle of the Shoulder Joint [units: radians]
            q11             : Angle of the Top Leg Servo Joint [units: radians]
            q12             : Angle of the Top Leg Elbow Joint [units: radians] (would be always 0)
            q13             : Angle to satisfy five-bar closed chain (q13 = q21 + q22 - q11 - q12) [units: radians] (would be always 0)
            q21             : Angle of the Bottom Leg Servo Joint [units: radians]
            q22             : Angle of the Bottom Leg Elbow Joint [units: radians] (would be always 0)
            qw1             : Angle of the First Wrist Servo Joint [units: radians]
            qw2             : Angle of the Second Wrist Servo Joint [units: radians]
            qw3             : Angle of the Third Wrist Servo Joint [units: radians]
            phi             : Angle of Orientation Constraint of Parallel Leg Mechanism
                              (i.e., phi = q21 + q22 = q11 + q12 + q13) [units: radians] (would be always 0)
        """


        rot_wrist3 = util.quaternion_2_rotation_matrix(wrist_quaternion)
        T_shoulder_wrist3 = np.eye(4, dtype=np.float32)
        T_shoulder_wrist3[0:3,0:3] = rot_wrist3
        T_shoulder_wrist3[0:3,3] = np.array(shoulder_2_toe_xyz).reshape(-1)

        P_shoulder_wrist = T_shoulder_wrist3[0:3,3].reshape(-1)

        First_3_joint = Leg.leg_ik_direct_calculation_3DoF(P_shoulder_wrist, which_leg, is_first_ik, prev_angles)

        shoulder_angle = First_3_joint[0]
        q11 = First_3_joint[1]
        q21 = First_3_joint[2]

        if q11 < -np.pi/2:
            q11 = -np.pi-q11
        if q11 > np.pi/2:
            q11 = np.pi-q11
        if q21 < 0:
            q21 = -q21



        T_0_shoulder = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 1, use_quaternion = False)
        T_0_wrist = Leg.leg_fk_direct_calculation(0.0, [shoulder_angle, q11, q21,0,0,0], 0, 11, use_quaternion = False)
        T_shoulder_wrist = np.dot(np.linalg.inv(T_0_shoulder), T_0_wrist)

        T_wrist_wrist3 = np.dot(np.linalg.inv(T_shoulder_wrist), T_shoulder_wrist3)

        rot_wrist = T_wrist_wrist3[0:3,0:3]

        # Rotation Matrix of the wrist orientation with respect to the Wrist Reference Frame (before 3-DoF wrist
        # rotations). This can be thought of as the rotation matrix of the wrist (so we determine what the 3-DoF
        # spherical joint angle need to be).

        r_w_11, r_w_12, r_w_13, \
            r_w_21, r_w_22, r_w_23, \
            r_w_31, r_w_32, r_w_33 = util.unpack_rotation_matrix(rot_wrist)

        # Trivial solution to the Spherical Joint
        qw2 = np.arctan2(-r_w_33, np.sqrt(r_w_13 ** 2 + r_w_23 ** 2))
        qw1 = np.arctan2(-r_w_23 / np.cos(qw2),  r_w_13 / np.cos(qw2))
        qw3 = np.arctan2(-r_w_31 / np.cos(qw2), -r_w_32 / np.cos(qw2))
        
        return [shoulder_angle, q11, 0, 0, q21, 0, qw1, qw2, qw3, 0]
