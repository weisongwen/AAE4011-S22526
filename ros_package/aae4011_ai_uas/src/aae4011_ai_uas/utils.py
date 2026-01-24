"""
AAE4011 - Utility Functions

Common utility functions used across the ROS package.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def sigmoid_function(z):
    """
    Compute the sigmoid (logistic) function.
    
    The sigmoid function: σ(z) = 1 / (1 + e^(-z))
    
    Parameters:
    -----------
    z : numpy array or float
        Input values
    
    Returns:
    --------
    sigmoid : numpy array or float
        Sigmoid function values
    """
    return 1 / (1 + np.exp(-z))


def quat_to_euler(quat):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Parameters:
    -----------
    quat : list or array
        Quaternion in [W, X, Y, Z] format
    
    Returns:
    --------
    euler : numpy array
        Euler angles [roll, pitch, yaw] in degrees
    """
    r = R.from_quat(quat)
    euler = r.as_euler('xyz', degrees=True)
    return euler


def normalize_time(time_array):
    """
    Normalize time array to start from 0 and convert to seconds.
    
    Parameters:
    -----------
    time_array : numpy array
        Time values (in nanoseconds)
    
    Returns:
    --------
    normalized : numpy array
        Normalized time values in seconds
    """
    return (time_array - time_array[0]) / 1e9
