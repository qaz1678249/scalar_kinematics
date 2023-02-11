import scaler_kin.v2.hardware_constants as old_hardware_constants
import numpy as np
from scaler_kin.v2.hardware_constants import division_factor, list_name_robots, list_name_fsms, list_name_devices, list_commands, list_interpolation_type, list_getCtrlPoint_fcn, SCALER_walking_consts

class SCALER_climbing_consts_6DoF(old_hardware_constants.SCALER_climbing_consts_6DoF):
    """ Class: SCALER_climbing_consts_6DoF
    This class contains the constant variables for the SCALER_v2 (Climbing Configuration w/ Gripper Kinematics)
    """
    # For controller parameters (admittance control)
    Md_const, Dd_const, Kd_const, Kf_const = 0.7, 250.0, 0.0, 1.0
    Md = np.diag(np.array([Md_const,Md_const,Md_const,Md_const,Md_const,Md_const]))
    Dd = np.diag(np.array([Dd_const,Dd_const,Dd_const,Dd_const,Dd_const,Dd_const]))
    Kd = np.diag(np.array([Kd_const,Kd_const,Kd_const,Kd_const,Kd_const,Kd_const]))
    Kf = np.diag(np.array([Kf_const,Kf_const,Kf_const,Kf_const,Kf_const,Kf_const]))
    # Length of the center parallel link (i.e., the Battery Link which is always parallel to the two Rigid Body Links)
    # [units: mm]
    L_BATTERY = 150

    # Length of the two parallel Body Links (i.e., the Body Links that change the body posture). [units: mm]
    L_BL = 100

    # Length from the Posture Joints to the Rigid Body Link (Measurement along the Y Body Frame axis) and the Length
    # from the Rigid Body Link to the Shoulder Joint (Measurement along the X Body Frame axis). In summary, this Length
    # produces a Rigid Body (L-Shaped) Structure connecting the Posture Joint to the Shoulder Joint. [units: mm]
    L_S2RBx = 86
    L_S2RBy = 69 

    # Leg numbers
    RIGHT_FRONT = 0
    RIGHT_BACK = 1
    LEFT_BACK = 2
    LEFT_FRONT = 3

    LEG_NUM = [RIGHT_FRONT, RIGHT_BACK, LEFT_BACK, LEFT_FRONT]  # previously legNo

    NUM_OF_LEGS = len(LEG_NUM)

    # TODO: Update the Motor ID to accommodate 6-DoF [lines 470-515]
    # MOTOR_ID: Assigned Motor ID's for the Dynamixel Motors

    # Leg 1
    # (1, 101): Right Front (Shoulder Motor (Pair))
    # (2, 102): Right Front (Top Leg Servo Motor (Pair))
    # (3, 103): Right Front (Bottom Leg Servo Motor (Pair))
    # 13: Right Front (Wrist Motor 1)
    # 14: Right Front (Wrist Motor 2)
    # 15: Right Front (Wrist Motor 3)

    # Leg 2
    # (4, 104): Right Back (Shoulder Motor (Pair))
    # (5, 105): Right Back (Top Leg Servo Motor (Pair))
    # (6, 106): Right Back (Bottom Leg Servo Motor (Pair))
    # 16: Right Back (Wrist Motor 1)
    # 17: Right Back (Wrist Motor 2)
    # 18: Right Back (Wrist Motor 3)

    # Leg 3
    # (7, 107): Left Back (Shoulder Motor (Pair))
    # (8, 108): Left Back (Top Leg Servo Motor (Pair))
    # (9, 109): Left Back (Bottom Leg Servo Motor (Pair))
    # 19: Left Back (Wrist Motor 1)
    # 20: Left Back (Wrist Motor 2)
    # 21: Left Back (Wrist Motor 3)

    # Leg 4
    # (10, 110): Left Front (Shoulder Motor (Pair))
    # (11, 111): Left Front (Top Leg Servo Motor (Pair))
    # (12, 112): Left Front (Bottom Leg Servo Motor (Pair))
    # 22: Left Front (Wrist Motor 1)
    # 23: Left Front (Wrist Motor 2)
    # 24: Left Front (Wrist Motor 3)

    # Body
    # 0: Body Posture Motor

    LEG_MASTER_MOTOR_ID = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 13, 14, 18, 16, 17, 21, 19, 20, 24, 22, 23]
    LEG_SLAVE_MOTOR_ID = [101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112]

    BODY_MOTOR_ID = [0]
    ALL_MOTOR_ID = BODY_MOTOR_ID + LEG_MASTER_MOTOR_ID + LEG_SLAVE_MOTOR_ID

    # TODO: Currently considering one for just one leg
    # Number of Linear Actuators per Leg
    LIN_MOTOR_ID = [0, 1, 2, 3]

    # Dimension of the Footstep (DoF) times 2 (since we have 2 fingertips)
    DIM_FOOTSTEP = 7  # previously dim_footstep

    # Dimension of the Finger Tips (Positon: XYZ + Orientaion Quaternion: wxyz)
    DIM_FINGER_TIPS = 7

    # Number of Fingers:
    DIM_NUM_FINGER = int(DIM_FOOTSTEP / DIM_FINGER_TIPS)

    # Number of Dynamixel Motors per Leg
    NUM_LEG_MOTOR = 6
    NUM_BODY_MOTOR = 1

    # Number of Gripper Linear Actuator Variables (Length of the Linear Actuator, Offset Angle of the Linear Actuator)
    NUM_GRIPPER_LINEAR_ACTUATOR_VARIABLES = 2


    # TODO: CHECK OFFSET!
    # Linear actuator offset value
    LINEAR_ACTUATOR_OFFSET = 75

    assert len(LEG_MASTER_MOTOR_ID) == (NUM_LEG_MOTOR * NUM_OF_LEGS), "Error: Inconsistent number of leg motors !"
    assert len(BODY_MOTOR_ID) == NUM_BODY_MOTOR, "Error: Inconsistent number of body motors !"

    # The X Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
    TOP_LEG_SERVO_OFF_X = -10.57

    # The Z Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
    TOP_LEG_SERVO_OFF_Z = -71.25

    # The X Offset from the Shoulder Servo to the Bottom Leg Servo. [units: mm]
    BOTTOM_LEG_SERVO_OFF_X = -10.57

    # The Z Offset from the Shoulder Servo to the Bottom Leg Servo. [units: mm]
    BOTTOM_LEG_SERVO_OFF_Z = -14.25*2 - 71.25

    # Length of the Wrist Link (equivalent to the length of the Gripper offset (z-component) from the Spherical Joint)
    # [units: mm]
    L_WRIST = 10

    # Length of the Gripper offset (x-component) from the Spherical Joint [units: mm]
    L_GRIPPER_OFFSET_X = 0

    # Length of the Gripper offset (y-component) from the Spherical Joint [units: mm]
    L_GRIPPER_OFFSET_Y = 45.85

    # Length of the Leg Link 1 (Links from the Top/Bottom Leg Servos to the Elbow Joints). [units: mm]
    L_LEG_LINK_1 = 113.00

    # Length of the Leg Link 2 (Links from the Elbow Joints to the Wrist Servo). [units: mm]
    L_LEG_LINK_2 = 142.035

    L_LEG_LINK_2_NEW = 136.35

    # Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the Position (X,Z) of
    # the Parallel Leg Origin with respect to Shoulder's Reference Frame) [units: mm]
    LEG_ORIGIN_X = (TOP_LEG_SERVO_OFF_X + BOTTOM_LEG_SERVO_OFF_X) / 2
    LEG_ORIGIN_Z = (TOP_LEG_SERVO_OFF_Z + BOTTOM_LEG_SERVO_OFF_Z) / 2

    # X and Z components of the vector difference between the Top and Bottom Servos of the Leg.
    BOTTOM_2_TOP_SERVO_VEC = np.array([(TOP_LEG_SERVO_OFF_X - BOTTOM_LEG_SERVO_OFF_X),
                                       0.0,
                                       (TOP_LEG_SERVO_OFF_Z - BOTTOM_LEG_SERVO_OFF_Z)])

    # Length Between the Leg Servos (Servo Pair) [units: mm]
    L_BLSP = np.linalg.norm(BOTTOM_2_TOP_SERVO_VEC)

    # Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset Positions of the Top and Bottom
    # Leg Servos). [units: radians]
    LEG_ROT_OFF_ANGLE = np.arctan2(BOTTOM_2_TOP_SERVO_VEC[0], BOTTOM_2_TOP_SERVO_VEC[2])

    L_BB = LEG_ORIGIN_X

    L_LEG_LINK_A23_A24 = 36.75

    L_LEG_LINK_A22_WRIST = 217.53
    
    L_WRIST_OFFSET_E = 59.13

    LEG_THETA_1_OFF_ANGLE = np.arcsin(L_LEG_LINK_A23_A24 / L_LEG_LINK_2)

    L_LEG_LINK_A22_A24 = L_LEG_LINK_2 * np.cos(LEG_THETA_1_OFF_ANGLE)
    L_LEG_LINK_A24_WRIST = L_LEG_LINK_A22_WRIST - L_LEG_LINK_A22_A24
    L_LEG_LINK_A23_WRIST = np.sqrt(L_LEG_LINK_A24_WRIST ** 2 + L_LEG_LINK_A23_A24 ** 2)

    LEG_THETA_2_OFF_ANGLE = np.arcsin(L_LEG_LINK_A23_A24 / L_LEG_LINK_A23_WRIST)

    LEG_GAMMA_OFF_ANGLE = LEG_THETA_1_OFF_ANGLE + LEG_THETA_2_OFF_ANGLE

    T_wrist_gripper_0and2 = np.array([[ 0, 1, 0, L_GRIPPER_OFFSET_X],
                                      [-1, 0, 0,-L_GRIPPER_OFFSET_Y],
                                      [ 0, 0, 1, -L_WRIST],
                                      [ 0,0, 0,1]])

    T_wrist_gripper_1and3 = np.array([[ 0,-1, 0, L_GRIPPER_OFFSET_X],
                                      [ 1, 0, 0, L_GRIPPER_OFFSET_Y],
                                      [ 0, 0, 1, -L_WRIST],
                                      [ 0,0, 0,1]])

    T_wrist_gripper_0and2_inv = np.linalg.inv(T_wrist_gripper_0and2)
    T_wrist_gripper_1and3_inv = np.linalg.inv(T_wrist_gripper_1and3)

    # TODO: Update the hardware offsets to account for the additional wrist motors (6-DoF)
    HARDWAREOFFSET = {
        'installation_trans': np.array(
            [[1,    # Shoulder Pair (Leg 1)
              -1,   # Top Leg Serve Pair
              1,    # Bottom Leg Servo Pair
              -1,    # First Wrist Servo
              -1,    # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 2)
              1,    # Top Leg Serve Pair
              -1,   # Bottom Leg Servo Pair
              -1,   # First Wrist Servo
              1,   # Second Wrist Servo
              -1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 3)
              -1,   # Top Leg Serve Pair
              1,    # Bottom Leg Servo Pair
              -1,    # First Wrist Servo
              1,    # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 4)
              1,    # Top Leg Serve Pair
              -1,   # Bottom Leg Servo Pair
              -1,   # First Wrist Servo
              -1,   # Second Wrist Servo
              -1]]), # Third Wrist Servo

        'installation_0': np.array(
            [[0.0,  # Identical for a pair of motors
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [0.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [0.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [0.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0]]) / 180.0 * np.pi}

    #########################################################
    # Hardware Constants for 2D Gripper (Two-Finger Design) #
    #########################################################

    #
    L_GRIPPER_2D_L_IJ_X = 116
    L_GRIPPER_2D_L_IJ_Y = 8

    L_GRIPPER_2D_L1 = 116

    L_GRIPPER_2D_L2 = 37.24
    L_GRIPPER_2D_L3 = 24.59
    L_GRIPPER_2D_L4 = 49.65
    L_GRIPPER_2D_L5 = L_GRIPPER_2D_L4
    L_GRIPPER_2D_L6 = L_GRIPPER_2D_L3
    L_GRIPPER_2D_L7 = L_GRIPPER_2D_L2

    L_GRIPPER_2D_L8 = 43.41
    L_GRIPPER_2D_L9 = 25.31
    L_GRIPPER_2D_L10 = 10.00
    L_GRIPPER_2D_L11 = L_GRIPPER_2D_L8
    L_GRIPPER_2D_L12 = L_GRIPPER_2D_L9
    L_GRIPPER_2D_L13 = L_GRIPPER_2D_L10

    L_GRIPPER_2D_L14 = 17.75
    L_GRIPPER_2D_L15 = L_GRIPPER_2D_L14

    # This class is just used to keep the constants
    Ripple = 0
    Amble = 1
    Tripod = 2
    Amble_Climb_1Wall = 3  # Amble gait for climbing on a single wall
    Tripod_BSpline = 5
    Tripod_Climb_2Wall_BSpline = 6  # Tripod gait for climbing between 2 walls with NURBS
    Amble_Climb_2Wall_BSpline = 7
    Ripple_Climb_2Wall_BSpline = 8
    Tripod_Switch_BSpline = 9  # Use this gait to switch btw wall climbing and walking
    Tripod_Climb_2Wall_Stiffness_Model_Verification = 10  # This gait is for verification of stiffness model,
    # it lifts leg 1,3,5 on the air

class SCALER_climbing_consts_6DoF_gripper(old_hardware_constants.SCALER_climbing_consts_6DoF_gripper):
    """ Class: SCALER_climbing_consts_6DoF
    This class contains the constant variables for the SCALER_v2 (Climbing Configuration w/ Gripper Kinematics)
    """
    # For controller parameters (admittance control)
    Md_const, Dd_const, Kd_const, Kf_const = 0.7, 250.0, 0.0, 1.0
    Md = np.diag(np.array([Md_const,Md_const,Md_const,Md_const,Md_const,Md_const]))
    Dd = np.diag(np.array([Dd_const,Dd_const,Dd_const,Dd_const,Dd_const,Dd_const]))
    Kd = np.diag(np.array([Kd_const,Kd_const,Kd_const,Kd_const,Kd_const,Kd_const]))
    Kf = np.diag(np.array([Kf_const,Kf_const,Kf_const,Kf_const,Kf_const,Kf_const]))
    # Length of the center parallel link (i.e., the Battery Link which is always parallel to the two Rigid Body Links)
    # [units: mm]
    L_BATTERY = 150

    # Length of the two parallel Body Links (i.e., the Body Links that change the body posture). [units: mm]
    L_BL = 100

    # Length from the Posture Joints to the Rigid Body Link (Measurement along the Y Body Frame axis) and the Length
    # from the Rigid Body Link to the Shoulder Joint (Measurement along the X Body Frame axis). In summary, this Length
    # produces a Rigid Body (L-Shaped) Structure connecting the Posture Joint to the Shoulder Joint. [units: mm]
    L_S2RBx = 86
    L_S2RBy = 69 

    # Leg numbers
    RIGHT_FRONT = 0
    RIGHT_BACK = 1
    LEFT_BACK = 2
    LEFT_FRONT = 3

    LEG_NUM = [RIGHT_FRONT, RIGHT_BACK, LEFT_BACK, LEFT_FRONT]  # previously legNo

    NUM_OF_LEGS = len(LEG_NUM)

    # TODO: Update the Motor ID to accommodate 6-DoF [lines 470-515]
    # MOTOR_ID: Assigned Motor ID's for the Dynamixel Motors

    # Leg 1
    # (1, 101): Right Front (Shoulder Motor (Pair))
    # (2, 102): Right Front (Top Leg Servo Motor (Pair))
    # (3, 103): Right Front (Bottom Leg Servo Motor (Pair))
    # 13: Right Front (Wrist Motor 1)
    # 14: Right Front (Wrist Motor 2)
    # 15: Right Front (Wrist Motor 3)

    # Leg 2
    # (4, 104): Right Back (Shoulder Motor (Pair))
    # (5, 105): Right Back (Top Leg Servo Motor (Pair))
    # (6, 106): Right Back (Bottom Leg Servo Motor (Pair))
    # 16: Right Back (Wrist Motor 1)
    # 17: Right Back (Wrist Motor 2)
    # 18: Right Back (Wrist Motor 3)

    # Leg 3
    # (7, 107): Left Back (Shoulder Motor (Pair))
    # (8, 108): Left Back (Top Leg Servo Motor (Pair))
    # (9, 109): Left Back (Bottom Leg Servo Motor (Pair))
    # 19: Left Back (Wrist Motor 1)
    # 20: Left Back (Wrist Motor 2)
    # 21: Left Back (Wrist Motor 3)

    # Leg 4
    # (10, 110): Left Front (Shoulder Motor (Pair))
    # (11, 111): Left Front (Top Leg Servo Motor (Pair))
    # (12, 112): Left Front (Bottom Leg Servo Motor (Pair))
    # 22: Left Front (Wrist Motor 1)
    # 23: Left Front (Wrist Motor 2)
    # 24: Left Front (Wrist Motor 3)

    # Body
    # 0: Body Posture Motor

    LEG_MASTER_MOTOR_ID = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
    LEG_SLAVE_MOTOR_ID = [101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112]

    BODY_MOTOR_ID = [0]
    ALL_MOTOR_ID = LEG_MASTER_MOTOR_ID + LEG_SLAVE_MOTOR_ID + BODY_MOTOR_ID
    ALL_MOTOR_ID.sort()

    # TODO: Currently considering one for just one leg
    # Number of Linear Actuators per Leg
    LIN_MOTOR_ID = [0, 1, 2, 3]

    # Dimension of the Footstep (DoF) times 2 (since we have 2 fingertips)
    DIM_FOOTSTEP = 14  # previously dim_footstep

    # Dimension of the Finger Tips (Positon: XYZ + Orientaion Quaternion: wxyz)
    DIM_FINGER_TIPS = 7

    # Number of Fingers:
    DIM_NUM_FINGER = int(DIM_FOOTSTEP / DIM_FINGER_TIPS)

    # Number of Dynamixel Motors per Leg
    NUM_LEG_MOTOR = 6
    NUM_BODY_MOTOR = 1

    # Number of Gripper Linear Actuator Variables (Length of the Linear Actuator, Offset Angle of the Linear Actuator)
    NUM_GRIPPER_LINEAR_ACTUATOR_VARIABLES = 2


    # TODO: CHECK OFFSET!
    # Linear actuator offset value
    LINEAR_ACTUATOR_OFFSET = 75

    assert len(LEG_MASTER_MOTOR_ID) == (NUM_LEG_MOTOR * NUM_OF_LEGS), "Error: Inconsistent number of leg motors !"
    assert len(BODY_MOTOR_ID) == NUM_BODY_MOTOR, "Error: Inconsistent number of body motors !"

    # The X Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
    TOP_LEG_SERVO_OFF_X = -10.57

    # The Z Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
    TOP_LEG_SERVO_OFF_Z = -71.25

    # The X Offset from the Shoulder Servo to the Bottom Leg Servo. [units: mm]
    BOTTOM_LEG_SERVO_OFF_X = -10.57

    # The Z Offset from the Shoulder Servo to the Bottom Leg Servo. [units: mm]
    BOTTOM_LEG_SERVO_OFF_Z = -14.25*2 - 71.25

    # Length of the Wrist Link (equivalent to the length of the Gripper offset (z-component) from the Spherical Joint)
    # [units: mm]
    L_WRIST = 10

    # Length of the Gripper offset (x-component) from the Spherical Joint [units: mm]
    L_GRIPPER_OFFSET_X = 0

    # Length of the Gripper offset (y-component) from the Spherical Joint [units: mm]
    L_GRIPPER_OFFSET_Y = 45.85

    # Length of the Leg Link 1 (Links from the Top/Bottom Leg Servos to the Elbow Joints). [units: mm]
    L_LEG_LINK_1 = 113.00

    # Length of the Leg Link 2 (Links from the Elbow Joints to the Wrist Servo). [units: mm]
    L_LEG_LINK_2 = 142.035

    L_LEG_LINK_2_NEW = 136.35

    # Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the Position (X,Z) of
    # the Parallel Leg Origin with respect to Shoulder's Reference Frame) [units: mm]
    LEG_ORIGIN_X = (TOP_LEG_SERVO_OFF_X + BOTTOM_LEG_SERVO_OFF_X) / 2
    LEG_ORIGIN_Z = (TOP_LEG_SERVO_OFF_Z + BOTTOM_LEG_SERVO_OFF_Z) / 2

    # X and Z components of the vector difference between the Top and Bottom Servos of the Leg.
    BOTTOM_2_TOP_SERVO_VEC = np.array([(TOP_LEG_SERVO_OFF_X - BOTTOM_LEG_SERVO_OFF_X),
                                       0.0,
                                       (TOP_LEG_SERVO_OFF_Z - BOTTOM_LEG_SERVO_OFF_Z)])

    # Length Between the Leg Servos (Servo Pair) [units: mm]
    L_BLSP = np.linalg.norm(BOTTOM_2_TOP_SERVO_VEC)

    # Rotation Angle Offset of the Parallel Leg Structure (Based on the X and Z Offset Positions of the Top and Bottom
    # Leg Servos). [units: radians]
    LEG_ROT_OFF_ANGLE = np.arctan2(BOTTOM_2_TOP_SERVO_VEC[0], BOTTOM_2_TOP_SERVO_VEC[2])

    L_BB = LEG_ORIGIN_X

    L_LEG_LINK_A23_A24 = 36.75

    L_LEG_LINK_A22_WRIST = 217.53
    
    L_WRIST_OFFSET_E = 59.13

    LEG_THETA_1_OFF_ANGLE = np.arcsin(L_LEG_LINK_A23_A24 / L_LEG_LINK_2)

    L_LEG_LINK_A22_A24 = L_LEG_LINK_2 * np.cos(LEG_THETA_1_OFF_ANGLE)
    L_LEG_LINK_A24_WRIST = L_LEG_LINK_A22_WRIST - L_LEG_LINK_A22_A24
    L_LEG_LINK_A23_WRIST = np.sqrt(L_LEG_LINK_A24_WRIST ** 2 + L_LEG_LINK_A23_A24 ** 2)

    LEG_THETA_2_OFF_ANGLE = np.arcsin(L_LEG_LINK_A23_A24 / L_LEG_LINK_A23_WRIST)

    LEG_GAMMA_OFF_ANGLE = LEG_THETA_1_OFF_ANGLE + LEG_THETA_2_OFF_ANGLE

    T_wrist_gripper_0and2 = np.array([[ 0, 1, 0, L_GRIPPER_OFFSET_X],
                                      [-1, 0, 0,-L_GRIPPER_OFFSET_Y],
                                      [ 0, 0, 1, -L_WRIST],
                                      [ 0,0, 0,1]])

    T_wrist_gripper_1and3 = np.array([[ 0,-1, 0, L_GRIPPER_OFFSET_X],
                                      [ 1, 0, 0, L_GRIPPER_OFFSET_Y],
                                      [ 0, 0, 1, -L_WRIST],
                                      [ 0,0, 0,1]])

    T_wrist_gripper_0and2_inv = np.linalg.inv(T_wrist_gripper_0and2)
    T_wrist_gripper_1and3_inv = np.linalg.inv(T_wrist_gripper_1and3)

    # TODO: Update the hardware offsets to account for the additional wrist motors (6-DoF)
    HARDWAREOFFSET = {
        'installation_trans': np.array(
            [[1,    # Shoulder Pair (Leg 1)
              -1,   # Top Leg Serve Pair
              1,    # Bottom Leg Servo Pair
              -1,    # First Wrist Servo
              -1,    # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 2)
              1,    # Top Leg Serve Pair
              -1,   # Bottom Leg Servo Pair
              -1,   # First Wrist Servo
              1,   # Second Wrist Servo
              -1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 3)
              -1,   # Top Leg Serve Pair
              1,    # Bottom Leg Servo Pair
              -1,    # First Wrist Servo
              1,    # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 4)
              1,    # Top Leg Serve Pair
              -1,   # Bottom Leg Servo Pair
              -1,   # First Wrist Servo
              -1,   # Second Wrist Servo
              -1]]), # Third Wrist Servo

        'installation_0': np.array(
            [[0.0,  # Identical for a pair of motors
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [0.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [0.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [0.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0]]) / 180.0 * np.pi}

    #########################################################
    # Hardware Constants for 2D Gripper (Two-Finger Design) #
    #########################################################

    #
    L_GRIPPER_2D_L_IJ_X = 116
    L_GRIPPER_2D_L_IJ_Y = 8

    L_GRIPPER_2D_L1 = 116

    L_GRIPPER_2D_L2 = 37.24
    L_GRIPPER_2D_L3 = 24.59
    L_GRIPPER_2D_L4 = 49.65
    L_GRIPPER_2D_L5 = L_GRIPPER_2D_L4
    L_GRIPPER_2D_L6 = L_GRIPPER_2D_L3
    L_GRIPPER_2D_L7 = L_GRIPPER_2D_L2

    L_GRIPPER_2D_L8 = 43.41
    L_GRIPPER_2D_L9 = 25.31
    L_GRIPPER_2D_L10 = 10.00
    L_GRIPPER_2D_L11 = L_GRIPPER_2D_L8
    L_GRIPPER_2D_L12 = L_GRIPPER_2D_L9
    L_GRIPPER_2D_L13 = L_GRIPPER_2D_L10

    L_GRIPPER_2D_L14 = 17.75
    L_GRIPPER_2D_L15 = L_GRIPPER_2D_L14

    # This class is just used to keep the constants
    Ripple = 0
    Amble = 1
    Tripod = 2
    Amble_Climb_1Wall = 3  # Amble gait for climbing on a single wall
    Tripod_BSpline = 5
    Tripod_Climb_2Wall_BSpline = 6  # Tripod gait for climbing between 2 walls with NURBS
    Amble_Climb_2Wall_BSpline = 7
    Ripple_Climb_2Wall_BSpline = 8
    Tripod_Switch_BSpline = 9  # Use this gait to switch btw wall climbing and walking
    Tripod_Climb_2Wall_Stiffness_Model_Verification = 10  # This gait is for verification of stiffness model,
    # it lifts leg 1,3,5 on the air

consts = {"SCALER_walking": SCALER_walking_consts, "SCALER_climbing_6DoF": SCALER_climbing_consts_6DoF, "SCALER_climbing_6DoF_gripper": SCALER_climbing_consts_6DoF_gripper}
