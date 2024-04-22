from scaler_kin.v2.SCALER_v2_Leg_6DOF_gripper import Leg
from scaler_kin.v2.SCALAR_kinematics import scaler_k as scaler_k_v2
from scaler_kin.v2.SCALAR_kinematics import scalar_k as scalar_k_v2
from warnings import warn

class scaler_k(scaler_k_v2):
    pass

class scalar_k(scalar_k_v2):
    def __new__(cls, *args, **kwargs):
        warn("Your import has name typo. use scaler_k instead of scalar_k", DeprecationWarning, stacklevel=2)
        return super().__new__(cls, *args, **kwargs)


