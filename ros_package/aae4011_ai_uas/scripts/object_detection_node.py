#!/usr/bin/env python3
"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk

Object Detection ROS Node using YOLOv5
This node subscribes to image topics and performs object detection using YOLOv5.
"""

import rospy
import torch
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import json


class ObjectDetectionNode:
    """
    ROS node for object detection using YOLOv5.
    
    Subscribes:
        - /camera/image_raw (sensor_msgs/Image): Input image
    
    Publishes:
        - /detections (std_msgs/String): JSON formatted detection results
        - /detection_image (sensor_msgs/Image): Image with bounding boxes
    
    Parameters:
        - image_topic: Input image topic
        - detection_topic: Output detection topic
        - model_name: YOLOv5 model name
        - confidence_threshold: Detection confidence threshold
    """
    
    def __init__(self):
        """Initialize the object detection node."""
        rospy.init_node('object_detection_node', anonymous=True)
        
        # Get parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.detection_topic = rospy.get_param('~detection_topic', '/detections')
        self.model_name = rospy.get_param('~model_name', 'yolov5s')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLOv5 model
        rospy.loginfo(f"Loading YOLOv5 model: {self.model_name}")
        try:
            self.model = torch.hub.load('ultralytics/yolov5', self.model_name, pretrained=True)
            self.model.conf = self.confidence_threshold
            rospy.loginfo("YOLOv5 model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv5 model: {e}")
            self.model = None
        
        # Publishers
        self.detection_pub = rospy.Publisher(self.detection_topic, String, queue_size=10)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo(f"Subscribed to: {self.image_topic}")
        rospy.loginfo(f"Publishing detections to: {self.detection_topic}")
    
    def image_callback(self, msg):
        """
        Callback for incoming images.
        
        Parameters:
        -----------
        msg : sensor_msgs/Image
            Input image message
        """
        if self.model is None:
            rospy.logwarn_throttle(10, "Model not loaded. Skipping detection.")
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run detection
            results = self.model(cv_image)
            
            # Get detection results as pandas DataFrame
            detections = results.pandas().xyxy[0]
            
            # Create detection message
            detection_msg = self.create_detection_message(detections)
            self.detection_pub.publish(detection_msg)
            
            # Publish image with bounding boxes
            rendered_image = results.render()[0]
            image_msg = self.bridge.cv2_to_imgmsg(rendered_image, "bgr8")
            self.image_pub.publish(image_msg)
            
            # Log detection summary
            people_count = len(detections[detections['name'] == 'person'])
            total_count = len(detections)
            rospy.loginfo_throttle(1, f"Detected {total_count} objects ({people_count} people)")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Detection error: {e}")
    
    def create_detection_message(self, detections):
        """
        Create a JSON-formatted detection message.
        
        Parameters:
        -----------
        detections : pandas.DataFrame
            Detection results from YOLOv5
        
        Returns:
        --------
        String : JSON formatted detection results
        """
        detection_list = []
        
        for idx, row in detections.iterrows():
            detection = {
                'class': row['name'],
                'confidence': float(row['confidence']),
                'bbox': {
                    'xmin': float(row['xmin']),
                    'ymin': float(row['ymin']),
                    'xmax': float(row['xmax']),
                    'ymax': float(row['ymax'])
                }
            }
            detection_list.append(detection)
        
        msg = String()
        msg.data = json.dumps({
            'timestamp': rospy.Time.now().to_sec(),
            'detections': detection_list,
            'total_count': len(detection_list),
            'people_count': len([d for d in detection_list if d['class'] == 'person'])
        })
        
        return msg
    
    def run(self):
        """Main loop."""
        rospy.loginfo("Object detection node started.")
        rospy.spin()


def main():
    """Main entry point."""
    try:
        node = ObjectDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
