#!/usr/bin/env python3
"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk

Trajectory Visualizer ROS Node
This node reads trajectory data from CSV files and publishes it for RViz visualization.
"""

import rospy
import pandas as pd
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R


class TrajectoryVisualizer:
    """
    ROS node for visualizing trajectory data from CSV files.
    
    Publishes:
        - /trajectory (nav_msgs/Path): Trajectory path for RViz
        - /trajectory_markers (visualization_msgs/MarkerArray): Markers for waypoints
    
    Parameters:
        - csv_file: Path to CSV file containing trajectory data
        - frame_id: Frame ID for published messages (default: 'world')
        - publish_rate: Publishing rate in Hz (default: 10)
    """
    
    def __init__(self):
        """Initialize the trajectory visualizer node."""
        rospy.init_node('trajectory_visualizer', anonymous=True)
        
        # Get parameters
        self.csv_file = rospy.get_param('~csv_file', '')
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.publish_rate = rospy.get_param('~publish_rate', 10)
        
        # Publishers
        self.path_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/trajectory_markers', MarkerArray, queue_size=10)
        
        # Load trajectory data
        self.trajectory_data = None
        self.path_msg = None
        
        if self.csv_file:
            self.load_trajectory()
        else:
            rospy.logwarn("No CSV file specified. Use ~csv_file parameter.")
    
    def load_trajectory(self):
        """Load trajectory data from CSV file."""
        try:
            rospy.loginfo(f"Loading trajectory from: {self.csv_file}")
            self.trajectory_data = pd.read_csv(self.csv_file)
            
            # Check required columns
            required_cols = ['PosX', 'PosY', 'PosZ']
            if not all(col in self.trajectory_data.columns for col in required_cols):
                rospy.logerr(f"CSV must contain columns: {required_cols}")
                return
            
            # Create Path message
            self.path_msg = self.create_path_message()
            rospy.loginfo(f"Loaded {len(self.trajectory_data)} trajectory points.")
            
        except Exception as e:
            rospy.logerr(f"Failed to load trajectory: {e}")
    
    def create_path_message(self):
        """Create a Path message from trajectory data."""
        path = Path()
        path.header.frame_id = self.frame_id
        
        for idx, row in self.trajectory_data.iterrows():
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.seq = idx
            
            # Position
            pose.pose.position.x = row['PosX']
            pose.pose.position.y = row['PosY']
            pose.pose.position.z = row['PosZ']
            
            # Orientation (if quaternion data available)
            if all(col in self.trajectory_data.columns for col in ['QuatW', 'QuatX', 'QuatY', 'QuatZ']):
                pose.pose.orientation.w = row['QuatW']
                pose.pose.orientation.x = row['QuatX']
                pose.pose.orientation.y = row['QuatY']
                pose.pose.orientation.z = row['QuatZ']
            else:
                pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        return path
    
    def create_markers(self):
        """Create marker array for visualization."""
        markers = MarkerArray()
        
        # Line strip marker for trajectory
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # Line width
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for idx, row in self.trajectory_data.iterrows():
            from geometry_msgs.msg import Point
            p = Point()
            p.x = row['PosX']
            p.y = row['PosY']
            p.z = row['PosZ']
            line_marker.points.append(p)
        
        markers.markers.append(line_marker)
        
        return markers
    
    def publish(self):
        """Publish trajectory data."""
        if self.path_msg is None:
            return
        
        # Update timestamp
        self.path_msg.header.stamp = rospy.Time.now()
        
        # Publish path
        self.path_pub.publish(self.path_msg)
        
        # Publish markers
        markers = self.create_markers()
        self.marker_pub.publish(markers)
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Trajectory visualizer started.")
        rospy.loginfo(f"Publishing to /trajectory at {self.publish_rate} Hz")
        
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


def main():
    """Main entry point."""
    try:
        visualizer = TrajectoryVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
