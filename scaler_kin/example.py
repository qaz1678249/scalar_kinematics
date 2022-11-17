from SCALAR_kinematics import scalar_k
import numpy as np
from time import perf_counter

my_scalar_k = scalar_k()

joint_angles = [0.1,0.1,np.pi/2-0.1,0.0,0.0,0.0]
which_leg = 0

fk_res = my_scalar_k.scalar_forward_kinematics(which_leg, joint_angles, with_body=True)

print(fk_res)

ik_res = my_scalar_k.scalar_inverse_kinematics(which_leg, fk_res, with_body=True)

N = int(2e4)
"""
st = perf_counter()
for _ in range(N):
    ik_res = my_scalar_k.scalar_inverse_kinematics(which_leg, fk_res, with_body=True)
et = perf_counter()-st

print(f"Joint Angel to FK: {joint_angles}")
print(f"Joint Angle Calculated from IK: {ik_res}")
print(f"Total Calculation time: {et} for {N} loop")
print(f"Time per Ik {et/N} s")
print(f"Time per Ik {et/N*1e3} ms")

print(ik_res)
"""

#fk_res = my_scalar_k.scalar_forward_kinematics(which_leg, joint_angles, with_body=False, with_gripper = True, L_actuator=105, theta_actuator=0)

#print(fk_res)

#ik_res = my_scalar_k.scalar_inverse_kinematics(which_leg, fk_res, with_body=False, with_gripper = True)

#print(ik_res)


joint_angles = [0.1,0.1,np.pi/2-0.1]

fk_res = my_scalar_k.scalar_forward_kinematics_3DoF(which_leg, joint_angles, with_body=True, output_xyz=True)

print(fk_res)

ik_res = my_scalar_k.scalar_inverse_kinematics_3DoF(which_leg, fk_res, with_body=True, input_xyz=True)

print(ik_res)


"""
###################
## old test loop ##
###################



from SCALAR_kinematics import scalar_k
import numpy as np
from time import perf_counter

my_scalar_k = scalar_k()

joint_angles = [0.2,0.3,np.pi/2,-0.5,1.1, -1.8]
which_leg = 0

fk_res = my_scalar_k.scalar_forward_kinematics(which_leg, joint_angles)

#print(fk_res)


N = int(2e5)

st = perf_counter()
for _ in range(N):
    ik_res = my_scalar_k.scalar_inverse_kinematics(which_leg, fk_res)
et = perf_counter()-st

print(f"Joint Angel to FK: {joint_angles}")
print(f"Joint Angle Calculated from IK: {ik_res}")
print(f"Total Calculation time: {et} for {N} loop")
print(f"Time per Ik {et/N} s")
print(f"Time per Ik {et/N*1e3} ms")

"""
