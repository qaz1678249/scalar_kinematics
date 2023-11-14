import numpy as np

from cam_arm_kin import Cam_arm

my_arm = Cam_arm()
joints = [1.1,1.1,1.1,0.3]

fk_res = my_arm.cam_arm_fk(joints)
print(fk_res)
px = fk_res[0,3]
py = fk_res[1,3]
pz = fk_res[2,3]
print(my_arm.cam_arm_ik(px,py,pz,2.5))
