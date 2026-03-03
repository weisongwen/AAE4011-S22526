#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GPS NLOS Dataset Generator

This script generates a synthetic dataset for GPS NLOS signal identification,
with features including SNR, Constellation, Elevation, and Azimuth.

Author: Dr. Weisong Wen
Department of Aeronautical and Aviation Engineering
The Hong Kong Polytechnic University
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import norm
from mpl_toolkits.mplot3d import Axes3D
import os

# Set plot style
plt.style.use('seaborn-v0_8-whitegrid')

# Create output directory for figures if it doesn't exist
if not os.path.exists('figures'):
    os.makedirs('figures')

def generate_sky_distribution(n_satellites=2000, min_elevation=5):
    """
    Generate a realistic distribution of satellite positions in the sky
    
    Parameters:
    n_satellites (int): Number of satellite observations to generate
    min_elevation (float): Minimum elevation angle in degrees
    
    Returns:
    tuple: (elevation angles, azimuth angles)
    """
    # Elevation angles follow a sinusoidal distribution
    # More satellites are visible at lower elevations
    elevation = np.rad2deg(np.arcsin(np.random.uniform(
        np.sin(np.deg2rad(min_elevation)), 1, n_satellites)))
    
    # Azimuth angles are uniformly distributed around the horizon
    azimuth = np.random.uniform(0, 360, n_satellites)
    
    return elevation, azimuth

def add_buildings(elevation, azimuth, building_directions, 
                  building_heights, building_widths):
    """
    Add simulated buildings that block satellites in specific directions
    
    Parameters:
    elevation (array): Satellite elevation angles
    azimuth (array): Satellite azimuth angles
    building_directions (list): List of azimuth directions where buildings are located
    building_heights (list): List of building heights in degrees of elevation
    building_widths (list): List of building widths in degrees of azimuth
    
    Returns:
    array: Boolean mask where True indicates the satellite is blocked by a building
    """
    n_satellites = len(elevation)
    is_blocked = np.zeros(n_satellites, dtype=bool)
    
    for direction, height, width in zip(building_directions, 
                                       building_heights, 
                                       building_widths):
        # Calculate angular distance to building direction (considering circular azimuth)
        az_distance = np.minimum(
            np.abs(azimuth - direction),
            360 - np.abs(azimuth - direction)
        )
        
        # Satellite is blocked if it's behind the building and below its height
        is_blocked = is_blocked | (
            (az_distance <= width / 2) & (elevation <= height)
        )
    
    return is_blocked

def generate_snr(elevation, constellation, is_nlos):
    """
    Generate realistic SNR values based on elevation, constellation, and NLOS status
    
    Parameters:
    elevation (array): Satellite elevation angles
    constellation (array): Satellite constellation indicators (1-4)
    is_nlos (array): Boolean indicator of NLOS status
    
    Returns:
    array: Simulated SNR values
    """
    n_satellites = len(elevation)
    
    # Base SNR increases with elevation angle
    base_snr = 25 + (20 * elevation / 90)
    
    # Add constellation-specific offsets
    constellation_offset = np.zeros(n_satellites)
    constellation_offset[constellation == 1] = 2  # GPS
    constellation_offset[constellation == 2] = 0  # GLONASS
    constellation_offset[constellation == 3] = 3  # Galileo
    constellation_offset[constellation == 4] = 1  # BeiDou
    
    # Add random variations
    snr = base_snr + constellation_offset + np.random.normal(0, 2, n_satellites)
    
    # Reduce SNR for NLOS signals
    nlos_reduction = np.random.uniform(5, 15, n_satellites)
    snr[is_nlos] -= nlos_reduction[is_nlos]
    
    # Add some noise to make the relationship non-linear
    snr += np.random.normal(0, 2, n_satellites)
    
    # Ensure SNR stays in a realistic range
    snr = np.clip(snr, 15, 55)
    
    return snr

def generate_urban_environment(n_samples=500):
    """
    Generate a dataset simulating an urban environment with buildings
    
    Parameters:
    n_samples (int): Number of satellite observations to generate
    
    Returns:
    DataFrame: DataFrame with satellite observations
    """
    # Generate satellite positions
    elevation, azimuth = generate_sky_distribution(n_samples)
    
    # Randomly assign constellations (1=GPS, 2=GLONASS, 3=Galileo, 4=BeiDou)
    constellation = np.random.choice([1, 2, 3, 4], size=n_samples, 
                                    p=[0.4, 0.3, 0.2, 0.1])
    
    # Define buildings that block signals
    building_directions = [45, 135, 225, 315]  # Buildings in NE, SE, SW, NW
    building_heights = [30, 40, 25, 35]  # Heights in degrees elevation
    building_widths = [60, 40, 50, 45]  # Widths in degrees azimuth
    
    # Determine NLOS status based on buildings
    is_blocked = add_buildings(elevation, azimuth, 
                              building_directions, 
                              building_heights, 
                              building_widths)
    
    # Add some randomness to NLOS state (some LOS signals might still be NLOS
    # due to smaller buildings, trees, etc. not explicitly modeled)
    for i in range(len(is_blocked)):
        # Low elevation satellites have a chance to be NLOS even if not blocked by major buildings
        if elevation[i] < 15 and not is_blocked[i]:
            if np.random.random() < 0.3:
                is_blocked[i] = True
        # Even some higher satellites could be NLOS
        elif elevation[i] < 30 and not is_blocked[i]:
            if np.random.random() < 0.1:
                is_blocked[i] = True
    
    # Generate SNR values
    snr = generate_snr(elevation, constellation, is_blocked)
    
    # Create DataFrame
    data = pd.DataFrame({
        'SNR': snr,
        'Constellation': constellation,
        'Elevation': elevation,
        'Azimuth': azimuth,
        'NLOS_Status': is_blocked.astype(int)
    })
    
    return data

def generate_suburban_environment(n_samples=300):
    """
    Generate a dataset simulating a suburban environment with fewer tall buildings
    
    Parameters:
    n_samples (int): Number of satellite observations to generate
    
    Returns:
    DataFrame: DataFrame with satellite observations
    """
    # Generate satellite positions
    elevation, azimuth = generate_sky_distribution(n_samples)
    
    # Randomly assign constellations (1=GPS, 2=GLONASS, 3=Galileo, 4=BeiDou)
    constellation = np.random.choice([1, 2, 3, 4], size=n_samples, 
                                    p=[0.4, 0.3, 0.2, 0.1])
    
    # Define buildings that block signals (fewer and lower than urban)
    building_directions = [90, 180, 270]  # Buildings in E, S, W
    building_heights = [20, 25, 15]  # Heights in degrees elevation
    building_widths = [30, 25, 35]  # Widths in degrees azimuth
    
    # Determine NLOS status based on buildings
    is_blocked = add_buildings(elevation, azimuth, 
                              building_directions, 
                              building_heights, 
                              building_widths)
    
    # Add some randomness to NLOS state (trees, small buildings, etc.)
    for i in range(len(is_blocked)):
        # Low elevation satellites have a chance to be NLOS even if not blocked by major buildings
        if elevation[i] < 15 and not is_blocked[i]:
            if np.random.random() < 0.2:
                is_blocked[i] = True
    
    # Generate SNR values
    snr = generate_snr(elevation, constellation, is_blocked)
    
    # Create DataFrame
    data = pd.DataFrame({
        'SNR': snr,
        'Constellation': constellation,
        'Elevation': elevation,
        'Azimuth': azimuth,
        'NLOS_Status': is_blocked.astype(int)
    })
    
    return data

def generate_open_sky_environment(n_samples=200):
    """
    Generate a dataset simulating an open sky environment with few obstructions
    
    Parameters:
    n_samples (int): Number of satellite observations to generate
    
    Returns:
    DataFrame: DataFrame with satellite observations
    """
    # Generate satellite positions
    elevation, azimuth = generate_sky_distribution(n_samples)
    
    # Randomly assign constellations (1=GPS, 2=GLONASS, 3=Galileo, 4=BeiDou)
    constellation = np.random.choice([1, 2, 3, 4], size=n_samples, 
                                    p=[0.4, 0.3, 0.2, 0.1])
    
    # In open sky, only very low elevation satellites might be NLOS
    is_blocked = np.zeros(n_samples, dtype=bool)
    for i in range(n_samples):
        if elevation[i] < 10:
            if np.random.random() < 0.15:
                is_blocked[i] = True
    
    # Generate SNR values
    snr = generate_snr(elevation, constellation, is_blocked)
    
    # Create DataFrame
    data = pd.DataFrame({
        'SNR': snr,
        'Constellation': constellation,
        'Elevation': elevation,
        'Azimuth': azimuth,
        'NLOS_Status': is_blocked.astype(int)
    })
    
    return data

def create_visualizations(combined_data):
    """
    Create various visualizations of the dataset
    
    Parameters:
    combined_data (DataFrame): The full dataset to visualize
    """
    # Plot SNR vs Elevation, colored by NLOS status
    plt.figure(figsize=(10, 6))
    sns.scatterplot(x='Elevation', y='SNR', hue='NLOS_Status', 
                   data=combined_data, alpha=0.7)
    plt.title('SNR vs Elevation Angle')
    plt.xlabel('Elevation Angle (degrees)')
    plt.ylabel('SNR (dB-Hz)')
    plt.legend(title='NLOS Status', labels=['LOS', 'NLOS'])
    plt.tight_layout()
    plt.savefig('figures/snr_vs_elevation.png', dpi=300)
    
    # Plot sky distribution using polar coordinates
    plt.figure(figsize=(10, 10))
    ax = plt.subplot(111, projection='polar')
    
    # Convert azimuth from degrees to radians
    azimuth_rad = np.deg2rad(combined_data['Azimuth'])
    
    # In polar plots, 90 degrees - elevation gives the radial distance
    radial_distance = 90 - combined_data['Elevation']
    
    # Plot
    scatter = ax.scatter(azimuth_rad, radial_distance, 
                       c=combined_data['NLOS_Status'], cmap='coolwarm',
                       alpha=0.7, s=30)
    
    # Set up the polar plot
    ax.set_theta_zero_location('N')  # 0 degrees at the top
    ax.set_theta_direction(-1)  # Clockwise
    ax.set_rlabel_position(0)
    ax.set_rticks([0, 30, 60, 90])  # From 90 degrees down to 0
    ax.set_rlim(0, 90)
    ax.set_yticklabels(['90°', '60°', '30°', '0°'])  # Elevation labels
    
    plt.colorbar(scatter, label='NLOS Status')
    plt.title('Sky Distribution of Satellites', y=1.08)
    plt.tight_layout()
    plt.savefig('figures/satellite_sky_distribution.png', dpi=300)
    
    # Create 3D visualization of SNR, Elevation, and Azimuth
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    scatter = ax.scatter(combined_data['Azimuth'], 
                        combined_data['Elevation'], 
                        combined_data['SNR'],
                        c=combined_data['NLOS_Status'], 
                        cmap='coolwarm',
                        s=30, alpha=0.7)
    
    ax.set_xlabel('Azimuth (degrees)')
    ax.set_ylabel('Elevation (degrees)')
    ax.set_zlabel('SNR (dB-Hz)')
    plt.colorbar(scatter, label='NLOS Status')
    plt.title('3D Visualization of Signal Features')
    plt.tight_layout()
    plt.savefig('figures/3d_signal_features.png', dpi=300)
    
    # Plot distribution of features by NLOS status
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
    
    # SNR distribution
    sns.histplot(data=combined_data, x='SNR', hue='NLOS_Status', 
                bins=20, ax=axs[0, 0], kde=True)
    axs[0, 0].set_title('SNR Distribution by NLOS Status')
    
    # Elevation distribution
    sns.histplot(data=combined_data, x='Elevation', hue='NLOS_Status', 
                bins=20, ax=axs[0, 1], kde=True)
    axs[0, 1].set_title('Elevation Distribution by NLOS Status')
    
    # Azimuth distribution
    sns.histplot(data=combined_data, x='Azimuth', hue='NLOS_Status', 
                bins=36, ax=axs[1, 0], kde=True)
    axs[1, 0].set_title('Azimuth Distribution by NLOS Status')
    
    # Constellation counts
    sns.countplot(data=combined_data, x='Constellation', hue='NLOS_Status', 
                 ax=axs[1, 1])
    axs[1, 1].set_title('NLOS Status by Constellation')
    axs[1, 1].set_xticks([0, 1, 2, 3])
    axs[1, 1].set_xticklabels(['GPS', 'GLONASS', 'Galileo', 'BeiDou'])
    
    plt.tight_layout()
    plt.savefig('figures/feature_distributions.png', dpi=300)
    
    # Pairplot for all features
    sns.pairplot(combined_data, hue='NLOS_Status', corner=True)
    plt.savefig('figures/feature_pairplot.png', dpi=300)
    
    # Calculate correlation matrix
    corr_matrix = combined_data.corr()
    print("\nCorrelation Matrix:")
    print(corr_matrix)
    
    # Plot correlation heatmap
    plt.figure(figsize=(10, 8))
    sns.heatmap(corr_matrix, annot=True, cmap='coolwarm', fmt='.2f')
    plt.title('Feature Correlation Matrix')
    plt.tight_layout()
    plt.savefig('figures/correlation_matrix.png', dpi=300)

def print_dataset_statistics(combined_data):
    """
    Print statistics and observations about the dataset
    
    Parameters:
    combined_data (DataFrame): The full dataset to analyze
    """
    print("Dataset Shape:", combined_data.shape)
    print("\nClass Distribution:")
    print(combined_data['NLOS_Status'].value_counts())
    print("\nSummary Statistics:")
    print(combined_data.describe())
    
    print("\nObservations about the synthetic dataset:")
    print(f"1. Total observations: {len(combined_data)}")
    print(f"2. NLOS signals: {combined_data['NLOS_Status'].sum()} " 
          f"({combined_data['NLOS_Status'].mean()*100:.1f}%)")
    print(f"3. Mean SNR for LOS signals: "
          f"{combined_data[combined_data['NLOS_Status']==0]['SNR'].mean():.1f} dB-Hz")
    print(f"4. Mean SNR for NLOS signals: "
          f"{combined_data[combined_data['NLOS_Status']==1]['SNR'].mean():.1f} dB-Hz")
    print(f"5. Mean elevation for LOS signals: "
          f"{combined_data[combined_data['NLOS_Status']==0]['Elevation'].mean():.1f}°")
    print(f"6. Mean elevation for NLOS signals: "
          f"{combined_data[combined_data['NLOS_Status']==1]['Elevation'].mean():.1f}°")

def main():
    """
    Main function to generate and save the dataset
    """
    print("Generating GPS NLOS Dataset...")
    
    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Generate data for different environments
    print("Generating urban environment data...")
    urban_data = generate_urban_environment(500)
    
    print("Generating suburban environment data...")
    suburban_data = generate_suburban_environment(300)
    
    print("Generating open sky environment data...")
    open_sky_data = generate_open_sky_environment(200)
    
    # Combine all datasets
    combined_data = pd.concat([urban_data, suburban_data, open_sky_data], ignore_index=True)
    
    # Print statistics
    print_dataset_statistics(combined_data)
    
    # Create visualizations
    print("Creating visualizations...")
    create_visualizations(combined_data)
    
    # Save to CSV file for use in other exercises
    combined_data.to_csv('gps_nlos_dataset.csv', index=False)
    print("\nDataset saved to 'gps_nlos_dataset.csv'")
    print("Visualizations saved to the 'figures' directory")

if __name__ == "__main__":
    main()