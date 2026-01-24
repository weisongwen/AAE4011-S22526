#!/usr/bin/env python3
"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk

Point Cloud Analyzer ROS Node
This node subscribes to 3D point cloud data and analyzes:
- Number of points in the point cloud
- Frequency of incoming point cloud messages
- Basic statistics (min, max, mean coordinates)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64, Int32
import sensor_msgs.point_cloud2 as pc2
import json
import time


class PointCloudAnalyzer:
    """
    ROS node for analyzing 3D point cloud data.
    
    Subscribes:
        - /velodyne_points or specified topic (sensor_msgs/PointCloud2): Input point cloud
    
    Publishes:
        - /pointcloud_stats (std_msgs/String): JSON formatted statistics
        - /pointcloud_num_points (std_msgs/Int32): Number of points
        - /pointcloud_frequency (std_msgs/Float64): Message frequency in Hz
    
    Parameters:
        - pointcloud_topic: Input point cloud topic (default: /velodyne_points)
        - window_size: Number of messages for frequency calculation (default: 10)
    """
    
    def __init__(self):
        """Initialize the point cloud analyzer node."""
        rospy.init_node('pointcloud_analyzer', anonymous=True)
        
        # =============================================================================
        # Get ROS parameters
        # =============================================================================
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/velodyne_points')
        self.window_size = rospy.get_param('~window_size', 10)
        
        # =============================================================================
        # Initialize variables for frequency calculation
        # =============================================================================
        self.timestamps = []  # Store timestamps for frequency calculation
        self.message_count = 0
        self.start_time = None
        
        # =============================================================================
        # Initialize statistics storage
        # =============================================================================
        self.last_num_points = 0
        self.last_frequency = 0.0
        self.total_points_received = 0
        
        # =============================================================================
        # Publishers
        # =============================================================================
        # Publish detailed statistics as JSON
        self.stats_pub = rospy.Publisher('/pointcloud_stats', String, queue_size=10)
        
        # Publish number of points
        self.num_points_pub = rospy.Publisher('/pointcloud_num_points', Int32, queue_size=10)
        
        # Publish frequency
        self.frequency_pub = rospy.Publisher('/pointcloud_frequency', Float64, queue_size=10)
        
        # =============================================================================
        # Subscriber
        # =============================================================================
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic, 
            PointCloud2, 
            self.pointcloud_callback,
            queue_size=1
        )
        
        # =============================================================================
        # Log initialization info
        # =============================================================================
        rospy.loginfo("=" * 60)
        rospy.loginfo("Point Cloud Analyzer Node Initialized")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Subscribed to: {self.pointcloud_topic}")
        rospy.loginfo(f"Frequency window size: {self.window_size} messages")
        rospy.loginfo("Publishing to:")
        rospy.loginfo("  - /pointcloud_stats (JSON statistics)")
        rospy.loginfo("  - /pointcloud_num_points (number of points)")
        rospy.loginfo("  - /pointcloud_frequency (Hz)")
        rospy.loginfo("=" * 60)
    
    def pointcloud_callback(self, msg):
        """
        Callback for incoming point cloud messages.
        
        Parameters:
        -----------
        msg : sensor_msgs/PointCloud2
            Input point cloud message
        """
        # =============================================================================
        # Record timestamp for frequency calculation
        # =============================================================================
        current_time = rospy.Time.now().to_sec()
        self.timestamps.append(current_time)
        self.message_count += 1
        
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = current_time
        
        # Keep only recent timestamps for frequency calculation
        if len(self.timestamps) > self.window_size:
            self.timestamps = self.timestamps[-self.window_size:]
        
        # =============================================================================
        # Calculate frequency
        # =============================================================================
        frequency = self.calculate_frequency()
        self.last_frequency = frequency
        
        # =============================================================================
        # Extract point cloud data
        # =============================================================================
        try:
            # Convert PointCloud2 to numpy array
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            num_points = len(points_list)
            self.last_num_points = num_points
            self.total_points_received += num_points
            
            # =============================================================================
            # Calculate point cloud statistics
            # =============================================================================
            if num_points > 0:
                points_array = np.array(points_list)
                
                # Basic statistics
                x_coords = points_array[:, 0]
                y_coords = points_array[:, 1]
                z_coords = points_array[:, 2]
                
                stats = {
                    'timestamp': current_time,
                    'message_count': self.message_count,
                    'num_points': num_points,
                    'frequency_hz': round(frequency, 2),
                    'total_points_received': self.total_points_received,
                    'x': {
                        'min': round(float(np.min(x_coords)), 3),
                        'max': round(float(np.max(x_coords)), 3),
                        'mean': round(float(np.mean(x_coords)), 3),
                        'std': round(float(np.std(x_coords)), 3)
                    },
                    'y': {
                        'min': round(float(np.min(y_coords)), 3),
                        'max': round(float(np.max(y_coords)), 3),
                        'mean': round(float(np.mean(y_coords)), 3),
                        'std': round(float(np.std(y_coords)), 3)
                    },
                    'z': {
                        'min': round(float(np.min(z_coords)), 3),
                        'max': round(float(np.max(z_coords)), 3),
                        'mean': round(float(np.mean(z_coords)), 3),
                        'std': round(float(np.std(z_coords)), 3)
                    },
                    'bounding_box': {
                        'x_range': round(float(np.max(x_coords) - np.min(x_coords)), 3),
                        'y_range': round(float(np.max(y_coords) - np.min(y_coords)), 3),
                        'z_range': round(float(np.max(z_coords) - np.min(z_coords)), 3)
                    }
                }
            else:
                stats = {
                    'timestamp': current_time,
                    'message_count': self.message_count,
                    'num_points': 0,
                    'frequency_hz': round(frequency, 2),
                    'total_points_received': self.total_points_received,
                    'warning': 'Empty point cloud received'
                }
            
            # =============================================================================
            # Publish statistics
            # =============================================================================
            self.publish_stats(stats, num_points, frequency)
            
            # =============================================================================
            # Log summary to console
            # =============================================================================
            rospy.loginfo_throttle(1.0, 
                f"Points: {num_points:,} | Freq: {frequency:.1f} Hz | "
                f"Total msgs: {self.message_count}"
            )
            
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
    
    def calculate_frequency(self):
        """
        Calculate the frequency of incoming messages.
        
        Returns:
        --------
        frequency : float
            Message frequency in Hz
        """
        if len(self.timestamps) < 2:
            return 0.0
        
        # Calculate frequency based on time difference between messages
        time_diffs = np.diff(self.timestamps)
        if len(time_diffs) > 0:
            avg_time_diff = np.mean(time_diffs)
            if avg_time_diff > 0:
                return 1.0 / avg_time_diff
        
        return 0.0
    
    def publish_stats(self, stats, num_points, frequency):
        """
        Publish statistics to ROS topics.
        
        Parameters:
        -----------
        stats : dict
            Statistics dictionary
        num_points : int
            Number of points in the point cloud
        frequency : float
            Message frequency in Hz
        """
        # Publish JSON statistics
        stats_msg = String()
        stats_msg.data = json.dumps(stats, indent=2)
        self.stats_pub.publish(stats_msg)
        
        # Publish number of points
        num_points_msg = Int32()
        num_points_msg.data = num_points
        self.num_points_pub.publish(num_points_msg)
        
        # Publish frequency
        freq_msg = Float64()
        freq_msg.data = frequency
        self.frequency_pub.publish(freq_msg)
    
    def run(self):
        """Main loop - keeps the node running."""
        rospy.loginfo("Point Cloud Analyzer started. Waiting for point cloud data...")
        rospy.spin()
    
    def shutdown_callback(self):
        """Callback for node shutdown."""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Point Cloud Analyzer Shutting Down")
        rospy.loginfo(f"Total messages received: {self.message_count}")
        rospy.loginfo(f"Total points processed: {self.total_points_received:,}")
        if self.message_count > 0:
            avg_points = self.total_points_received / self.message_count
            rospy.loginfo(f"Average points per message: {avg_points:,.0f}")
        rospy.loginfo("=" * 60)


def main():
    """Main entry point."""
    try:
        analyzer = PointCloudAnalyzer()
        rospy.on_shutdown(analyzer.shutdown_callback)
        analyzer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
