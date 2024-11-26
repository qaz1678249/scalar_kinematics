from scaler_kin.biped import Leg
import numpy as np
from scipy.spatial.transform import Rotation as R


my_scalar_k = Leg()

# Define joint angles
joint_angles = [0.0, 0.4, 1.23]
#joint_angles = [0.0, 0.0, 0.0]  # Overwriting with zeros as per your code

which_leg = 1
delta_j = 0.01  # Small joint angle increment

# Compute the Jacobian at the initial joint angles (convert mm to meters)
jac_res = my_scalar_k.leg_jacobian_3DoF(joint_angles, which_leg) / 1000.0  # Jacobian in meters

# Compute the condition number of the Jacobian
# Singular Value Decomposition (SVD) of the Jacobian
U, S, Vt = np.linalg.svd(jac_res)
condition_number = S[0] / S[-1]
print("Condition Number of the Jacobian:", condition_number)

# Forward kinematics at initial joint angles
fk_res_old = my_scalar_k.leg_fk_direct_calculation(0.0, joint_angles + [0, 0, 0], which_leg, 11, use_quaternion=False)
fk_res_old = fk_res_old[0:3, 3] / 1000.0  # Position in meters

# Update joint angles
joint_angles = np.array(joint_angles) + delta_j
joint_angles = joint_angles.tolist()

# Forward kinematics at new joint angles
fk_res_new = my_scalar_k.leg_fk_direct_calculation(0.0, joint_angles + [0, 0, 0], which_leg, 11, use_quaternion=False)
fk_res_new = fk_res_new[0:3, 3] / 1000.0  # Position in meters

# Compute the change in position
delta_p = fk_res_new - fk_res_old

# Compute the change in position using the Jacobian
delta_p_jac = np.dot(jac_res, np.array([delta_j, delta_j, delta_j]).reshape((3, 1)))

print("Delta position from FK:", delta_p)
print("Delta position from Jacobian:", delta_p_jac.flatten())
print("Difference:", (delta_p - delta_p_jac.flatten()).flatten())

# Recompute the Jacobian at new joint angles
jac_res = my_scalar_k.leg_jacobian_3DoF(joint_angles, which_leg) / 1000.0  # Jacobian in meters

# Compute the manipulability matrix (for velocity ellipsoid)
manipulability_matrix = np.dot(jac_res, jac_res.T)

# Compute the force ellipsoid matrix (inverse of manipulability matrix)
force_ellipsoid_matrix = np.linalg.inv(manipulability_matrix)

# Compute eigenvalues and eigenvectors for the velocity ellipsoid
vel_eigenvalues, vel_eigenvectors = np.linalg.eig(manipulability_matrix)
vel_axes_lengths = np.sqrt(vel_eigenvalues)  # Semi-axis lengths for velocity ellipsoid

# Compute the volume of the velocity ellipsoid
a_v, b_v, c_v = vel_axes_lengths
velocity_ellipsoid_volume = np.sqrt(a_v * b_v * c_v)
print("Velocity Ellipsoid Axes Lengths:", vel_axes_lengths)
print("Velocity Ellipsoid Volume:", velocity_ellipsoid_volume)

# Compute eigenvalues and eigenvectors for the force ellipsoid
force_eigenvalues, force_eigenvectors = np.linalg.eig(force_ellipsoid_matrix)
# Compute the axes lengths for the force ellipsoid
force_axes_lengths = 1 / vel_axes_lengths

# Compute the volume of the force ellipsoid
a_f, b_f, c_f = force_axes_lengths
# Compute the volume of the force ellipsoid
a_f, b_f, c_f = force_axes_lengths
force_ellipsoid_volume = np.sqrt(a_f * b_f * c_f)
print("Force Ellipsoid Axes Lengths:", force_axes_lengths)
print("Force Ellipsoid Volume:", force_ellipsoid_volume)

# Compute the ratio of the volumes of the velocity and force ellipsoids
volume_ratio = velocity_ellipsoid_volume / force_ellipsoid_volume
print("Volume Ratio (Velocity / Force):", volume_ratio)
