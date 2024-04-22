from scaler_kin.v2.hardware_constants import SCALER_climbing_consts_6DoF_gripper, SCALER_walking_consts
from scaler_kin.v2.hardware_constants import SCALER_climbing_consts_6DoF as climbing

import numpy as np
class SCALER_climbing_consts_6DoF(climbing):
    # The X Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
    TOP_LEG_SERVO_OFF_X = np.array([-10.5, -10.25, -10.25, -10.5])

    # The Z Offset from the Shoulder Servo to the Top Leg Servo. [units: mm]
    TOP_LEG_SERVO_OFF_Z = np.array([-71.25, -71.25 - 2.50, -71.25 - 2.50, -71.25])

    # The X Offset from the Shoulder Servo to the Bottom Leg Servo. [units: mm]
    BOTTOM_LEG_SERVO_OFF_X = np.array([-10.5, -10.25, -10.25, -10.5])

    # The Z Offset from the Shoulder Servo to the Bottom Leg Servo. [units: mm]
    BOTTOM_LEG_SERVO_OFF_Z = np.array([-28.5 - 71.25, -33.5 - 71.25,  -33.5 - 71.25, -28.5 - 71.25])


    # Length of the Leg Link 1 (Links from the Top/Bottom Leg Servos to the Elbow Joints). [units: mm]
    L_LEG_LINK_1 = np.array([113.00, 130.0, 130.0, 113.0])

    # Center Point Between the Top Leg Servo and the Bottom Leg Servo. (These X and Z represent the Position (X,Z) of
    # the Parallel Leg Origin with respect to Shoulder's Reference Frame) [units: mm]
    LEG_ORIGIN_X = (TOP_LEG_SERVO_OFF_X + BOTTOM_LEG_SERVO_OFF_X) / 2
    LEG_ORIGIN_Z = (TOP_LEG_SERVO_OFF_Z + BOTTOM_LEG_SERVO_OFF_Z) / 2

    # X and Z components of the vector difference between the Top and Bottom Servos of the Leg.



consts = {"SCALER_walking": SCALER_walking_consts, "SCALER_climbing_6DoF": SCALER_climbing_consts_6DoF, "SCALER_climbing_6DoF_gripper": SCALER_climbing_consts_6DoF_gripper}
