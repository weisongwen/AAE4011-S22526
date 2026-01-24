"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026
Week 3: Linear & Logistic Regression

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk
Address: R820, PolyU

This code is provided for educational purposes as part of the AAE4011 course.
All rights reserved.

EvaPos.py - Trajectory Evaluation and Position Plotting
This script loads trajectory data from CSV files and visualizes position data
including X-Y trajectory, and position over time in X, Y, and Z directions.

Note: This code can also be run on Google Colab.
Google Colab: https://colab.research.google.com/
"""

# =============================================================================
# Import required libraries
# =============================================================================
import pandas as pd                              # For data manipulation and CSV reading
import matplotlib.pyplot as plt                   # For plotting and visualization
from mpl_toolkits.mplot3d import Axes3D          # For 3D plotting (if needed)
from scipy.spatial.transform import Rotation as R # For quaternion to Euler conversion

# =============================================================================
# File paths configuration
# CSV files should be in the same folder as this script
# =============================================================================
file_path1 = 'Solution1.csv'  # First trajectory data (Baseline)
file_path2 = 'Solution2.csv'  # Second trajectory data (Proposed method)

# =============================================================================
# Load and preprocess data from CSV files
# =============================================================================

# Load first CSV file (Baseline trajectory)
data1 = pd.read_csv(file_path1)

# Load second CSV file (Proposed trajectory)
data2 = pd.read_csv(file_path2)

# =============================================================================
# Process Dataset 1 (Baseline)
# =============================================================================

# Store original time and normalize time to start from 0
# Time is converted from nanoseconds to seconds
time1 = data1['Time']
data1['Time'] = (data1['Time'] - time1.iloc[0]) / (1e09)

# Extract position data (X, Y, Z coordinates)
pos1 = data1[['PosX', 'PosY', 'PosZ']]

# Extract velocity data (X, Y, Z components)
vel1 = data1[['VelX', 'VelY', 'VelZ']]

# Extract quaternion data (orientation as W, X, Y, Z)
quat1 = data1[['QuatW', 'QuatX', 'QuatY', 'QuatZ']]

# =============================================================================
# Process Dataset 2 (Proposed method)
# =============================================================================

# Store original time and normalize time to start from 0
# Time is converted from nanoseconds to seconds
time2 = data2['Time']
data2['Time'] = (data2['Time'] - time2.iloc[0]) / (1e09)

# Extract position data (X, Y, Z coordinates)
pos2 = data2[['PosX', 'PosY', 'PosZ']]

# Extract velocity data (X, Y, Z components)
vel2 = data2[['VelX', 'VelY', 'VelZ']]

# Extract quaternion data (orientation as W, X, Y, Z)
quat2 = data2[['QuatW', 'QuatX', 'QuatY', 'QuatZ']]

# =============================================================================
# Quaternion to Euler angle conversion
# =============================================================================

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

# Convert quaternions to Euler angles for both datasets
# Apply the conversion function row by row
euler1 = quat1.apply(lambda row: quat_to_euler([row['QuatW'], row['QuatX'], row['QuatY'], row['QuatZ']]), axis=1)
euler2 = quat2.apply(lambda row: quat_to_euler([row['QuatW'], row['QuatX'], row['QuatY'], row['QuatZ']]), axis=1)

# Convert the results to DataFrames with proper column names
euler1 = pd.DataFrame(euler1.tolist(), columns=['Roll', 'Pitch', 'Yaw'])
euler2 = pd.DataFrame(euler2.tolist(), columns=['Roll', 'Pitch', 'Yaw'])

# =============================================================================
# Visualization - Create 2x2 subplot figure
# =============================================================================

# Create a figure with four subplots (2 rows, 2 columns)
fig, axs = plt.subplots(2, 2, figsize=(15, 10))

# -----------------------------------------------------------------------------
# Subplot 1 (Top-Left): X-Y Trajectory
# Shows the 2D trajectory in the horizontal plane
# -----------------------------------------------------------------------------
axs[0, 0].plot(pos1['PosX'], pos1['PosY'], marker='', linestyle='-', color='r', label='Baseline')
# Uncomment the line below to also plot the proposed trajectory
# axs[0, 0].plot(pos2['PosX'], pos2['PosY'], marker='', linestyle='-', color='b', label='Proposed')
axs[0, 0].set_title('Trajectory (X-Y)')
axs[0, 0].set_xlabel('X direction')
axs[0, 0].set_ylabel('Y direction')
axs[0, 0].legend()
axs[0, 0].grid(True)

# -----------------------------------------------------------------------------
# Subplot 2 (Top-Right): X Position vs Time
# Shows how the X coordinate changes over time
# -----------------------------------------------------------------------------
axs[0, 1].plot(data1['Time'], pos1['PosX'], marker='', linestyle='-', color='r', label='Baseline')
axs[0, 1].plot(data2['Time'], pos2['PosX'], marker='', linestyle='-', color='b', label='Proposed')
axs[0, 1].set_title('X direction')
axs[0, 1].set_xlabel('time')
axs[0, 1].set_ylabel('X')
axs[0, 1].legend()
axs[0, 1].grid(True)

# -----------------------------------------------------------------------------
# Subplot 3 (Bottom-Left): Y Position vs Time
# Shows how the Y coordinate changes over time
# -----------------------------------------------------------------------------
axs[1, 0].plot(data1['Time'], pos1['PosY'], marker='', linestyle='-', color='r', label='Baseline')
axs[1, 0].plot(data2['Time'], pos2['PosY'], marker='', linestyle='-', color='b', label='Proposed')
axs[1, 0].set_title('Y direction')
axs[1, 0].set_xlabel('time')
axs[1, 0].set_ylabel('Y')
axs[1, 0].legend()
axs[1, 0].grid(True)

# -----------------------------------------------------------------------------
# Subplot 4 (Bottom-Right): Z Position vs Time
# Shows how the Z coordinate (altitude) changes over time
# -----------------------------------------------------------------------------
axs[1, 1].plot(data1['Time'], pos1['PosZ'], marker='', linestyle='-', color='r', label='Baseline')
axs[1, 1].plot(data2['Time'], pos2['PosZ'], marker='', linestyle='-', color='b', label='Proposed')
axs[1, 1].set_title('Z direction')
axs[1, 1].set_xlabel('time')
axs[1, 1].set_ylabel('Z')
axs[1, 1].legend()
axs[1, 1].grid(True)

# =============================================================================
# Final adjustments and output
# =============================================================================

# Adjust layout to prevent subplot overlap
plt.tight_layout()

# Save the plot as an image file (PNG format)
plt.savefig('four_subplots.png')
print("Plot saved as 'four_subplots.png'")

# Display the plot
plt.show()
