# SCALAR kinematics (ver 0.3.*)


SCALAR_kinematics.py is the orgnized class for 6DoF or 3DoF or with gripper FK and IK.

see example.py how to use the code

## Install the latest version
```bash
pip install git+https://github.com/qaz1678249/scalar_kinematics.git
```

## How to import
```python
from scaler_kin import scaler_k  # imports the latest scaler kinematics
from scaler_kin import Leg # import the leg class for 6DoF or with gripper
```


## import older or different kinematics configurations
```python
from scaler_kin.v2 import scaler_k  # imports the older v2 scaler kinematics
from scaler_kin.v2 import Leg  # imports the older leg class for 6DoF or with gripper
```

***
## Change since Ver 0.2 
The way importing classes have changed to a proper syntax. 
especially:
* Leg
* scaler_k
* hardware_constants
***
## Install Ver 0.2 (Only for Compatibility)

If you need to use older way of imports you can use ver_0.2 branch

```bash
pip install git+https://github.com/qaz1678249/scalar_kinematics.git@ver_0.2
```
