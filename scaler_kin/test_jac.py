from scaler_kin.biped import Leg
import numpy as np
from scipy.spatial.transform import Rotation as R

# Initialize the leg
my_scalar_k = Leg()

# Define joint angles
joint_angles = [0.0, 0.4, 1.23]
# joint_angles = [0.0, 0.0, 0.0]  # Overwriting with zeros as per your code

which_leg = 0
delta_j = 0.01  # Small joint angle increment

# Define joint velocities and torques
joint_velocities = np.array([1.0, 1.0, 1.0])  # Adjust units as necessary
joint_torques = np.array([1.0, 1.0, 1.0])    # Adjust units as necessary

# Compute the Jacobian at the initial joint angles (convert mm to meters)
jac_res = my_scalar_k.leg_jacobian_3DoF(joint_angles, which_leg) / 1000.0  # Jacobian in meters

# Compute the condition number of the Jacobian
# Singular Value Decomposition (SVD) of the Jacobian
U, S, Vt = np.linalg.svd(jac_res)
condition_number = S[0] / S[-1]
print("Condition Number of the Jacobian:", condition_number)

# Compute the manipulability matrix (for velocity ellipsoid)
manipulability_matrix = np.dot(jac_res, jac_res.T)

# Compute the force ellipsoid matrix (inverse of manipulability matrix)
force_ellipsoid_matrix = np.linalg.inv(manipulability_matrix)

# Compute eigenvalues and eigenvectors for the velocity ellipsoid
vel_eigenvalues, vel_eigenvectors = np.linalg.eig(manipulability_matrix)
vel_axes_lengths = np.sqrt(vel_eigenvalues)  # Semi-axis lengths for velocity ellipsoid

# Compute the volume of the velocity ellipsoid
a_v, b_v, c_v = vel_axes_lengths
velocity_ellipsoid_volume = (4 / 3) * np.pi * a_v * b_v * c_v  # Volume of an ellipsoid: (4/3)*π*a*b*c
print("Velocity Ellipsoid Axes Lengths (meters):", vel_axes_lengths)
print("Velocity Ellipsoid Volume (m³):", velocity_ellipsoid_volume)

# Compute eigenvalues and eigenvectors for the force ellipsoid
force_eigenvalues, force_eigenvectors = np.linalg.eig(force_ellipsoid_matrix)
# Compute the axes lengths for the force ellipsoid
force_axes_lengths = 1 / vel_axes_lengths

# Compute the volume of the force ellipsoid
a_f, b_f, c_f = force_axes_lengths
force_ellipsoid_volume = (4 / 3) * np.pi * a_f * b_f * c_f  # Volume of an ellipsoid: (4/3)*π*a*b*c
print("Force Ellipsoid Axes Lengths (meters):", force_axes_lengths)
print("Force Ellipsoid Volume (m³):", force_ellipsoid_volume)

# Compute the ratio of the volumes of the velocity and force ellipsoids
volume_ratio = velocity_ellipsoid_volume / force_ellipsoid_volume
print("Volume Ratio (Velocity / Force):", volume_ratio)

# Calculate end-effector velocity
end_effector_velocity = np.dot(jac_res, joint_velocities)
print("End-Effector Velocity (m/s):", end_effector_velocity)

# Calculate end-effector force
end_effector_force = np.dot(jac_res.T, joint_torques)
print("End-Effector Force (N):", end_effector_force)


# --- New Code: Calculate Euler Angles of the Ellipsoids ---

def eigenvectors_to_euler_angles(eigenvectors, sequence='xyz'):
    """
    Convert eigenvectors to Euler angles.

    Parameters:
    - eigenvectors: 3x3 matrix where each column is an eigenvector.
    - sequence: Euler angle sequence, e.g., 'xyz', 'zyx', etc.

    Returns:
    - Euler angles in degrees.
    """
    rotation_matrix = eigenvectors
    rotation = R.from_matrix(rotation_matrix)
    euler_angles = rotation.as_euler(sequence, degrees=True)
    return euler_angles


# Calculate Euler angles for the velocity ellipsoid
# Ensure that eigenvectors form a right-handed coordinate system
if np.linalg.det(vel_eigenvectors) < 0:
    vel_eigenvectors[:, 0] = -vel_eigenvectors[:, 0]

vel_euler_angles = eigenvectors_to_euler_angles(vel_eigenvectors, sequence='xyz')
print("Velocity Ellipsoid Euler Angles (degrees):", vel_euler_angles)

# Calculate Euler angles for the force ellipsoid
# Ensure that eigenvectors form a right-handed coordinate system
if np.linalg.det(force_eigenvectors) < 0:
    force_eigenvectors[:, 0] = -force_eigenvectors[:, 0]

force_euler_angles = eigenvectors_to_euler_angles(force_eigenvectors, sequence='xyz')
print("Force Ellipsoid Euler Angles (degrees):", force_euler_angles)
