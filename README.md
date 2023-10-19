# SCALAR kinematics (ver 0.4.*)
SCALAR_kinematics.py is the organized class for 6DoF, 4DoF or 3DoF or with gripper FK and IK.

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


## import original or different kinematics configurations
```python
from scaler_kin.v2 import scaler_k  # imports the v2 scaler kinematics
from scaler_kin.v2 import Leg  # imports the leg class for 6DoF or with gripper
```

## Change since Ver. 0.4
### 4DoF FK and IK
4DoF kinematics is added 

### Deprecated wrong class and function names
the kinematics class was wrongly named as scalar_k instead of scaler_k as well as their function names. 

Now the following import works, but shows you deprecated warning
```python
from scaler_kin import scalar_k  # imports the latest scaler kinematics
s = scalar_k()
s.scalar_forward_kinematics(...)
```
So instead, use 
```python
from scaler_kin import scaler_k  # imports the latest scaler kinematics
s = scaler_k()
s.scaler_forward_kinematics(...)
```


---
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
