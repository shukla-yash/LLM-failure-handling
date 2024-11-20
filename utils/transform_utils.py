from scipy.spatial.transform import Rotation as R

def correct_quaternion_ignore_roll(quat):
    """
    Correct the given quaternion by ignoring the roll (rotation around the x-axis).
    
    Args:
    quat (list or np.array): Quaternion as [x, y, z, w].
    
    Returns:
    np.array: Corrected quaternion with roll set to zero.
    """
    # Create a Rotation object from the quaternion
    r = R.from_quat(quat)
    
    # Convert to Euler angles (roll, pitch, yaw)
    euler = r.as_euler('xyz', degrees=False)
    
    # Set roll to zero to ignore rotation around x-axis
    euler[0] = 0.0  # Zero out roll
    euler[1] = 0.0  # Zero out pitch
    
    # Convert the modified Euler angles back to a quaternion
    corrected_quat = R.from_euler('xyz', euler).as_quat()
    
    return corrected_quat