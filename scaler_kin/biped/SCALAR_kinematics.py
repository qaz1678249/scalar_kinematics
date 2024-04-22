from .SCALER_v2_Leg_6DOF_gripper import Leg
from scaler_kin.v3.SCALAR_kinematics import scaler_k as scaler_k_v3
from scaler_kin.v3.SCALAR_kinematics import scalar_k as scalar_k_v3
from warnings import warn

class scaler_k(scaler_k_v3):
    def __init__(self): # overwrite the init to import the v2 Leg() instead
        self.k_model = Leg()


class scalar_k(scalar_k_v3):
    def __new__(cls, *args, **kwargs):
        warn("Your import has name typo. use scaler_k instead of scalar_k", DeprecationWarning, stacklevel=2)
        return super().__new__(cls, *args, **kwargs)
    def __init__(self): # overwrite the init to import the v2 Leg() instead
        self.k_model = Leg()
        self.scaler_k = scaler_k()

