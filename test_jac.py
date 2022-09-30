from SCALER_v2_Leg_6DOF_gripper import Leg
import numpy as np

my_scalar_k = Leg()

joint_angles = [0.0,0.0,np.pi/2]
which_leg = 0
delta_j = 0.001

jac_res = my_scalar_k.leg_jacobian_3DoF(joint_angles, which_leg)
fk_res_old = my_scalar_k.leg_fk_direct_calculation(0.0,joint_angles+[0,0,0],which_leg,11,use_quaternion = False)
fk_res_old = fk_res_old[0:3,3]



joint_angles = np.array(joint_angles) + delta_j
joint_angles = joint_angles.tolist()

fk_res_new = my_scalar_k.leg_fk_direct_calculation(0.0,joint_angles+[0,0,0],which_leg,11,use_quaternion = False)
fk_res_new = fk_res_new[0:3,3]

delta_p = fk_res_new - fk_res_old

delta_p_jac = np.dot(jac_res, np.array([delta_j, delta_j, delta_j]).reshape((3,1)))

print(delta_p)
print(delta_p_jac)
print(delta_p.reshape(-1)-delta_p_jac.reshape(-1))

