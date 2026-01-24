#!/usr/bin/env python3
"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk

Logistic Regression ROS Node
This node demonstrates logistic regression for binary classification in a ROS environment.
"""

import rospy
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, String
import json


def sigmoid_function(z):
    """
    Compute the sigmoid (logistic) function.
    
    Parameters:
    -----------
    z : numpy array or float
        Input values
    
    Returns:
    --------
    sigmoid : numpy array or float
        Sigmoid function values (probability)
    """
    return 1 / (1 + np.exp(-z))


class LogisticRegressionNode:
    """
    ROS node demonstrating logistic regression classification.
    
    Subscribes:
        - /input_features (std_msgs/Float64MultiArray): Input feature vector
    
    Publishes:
        - /classification_result (std_msgs/String): Classification result (JSON)
        - /probability (std_msgs/Float64): Probability of class 1
    
    Parameters:
        - slope: Slope parameter for logistic function
        - intercept: Intercept parameter for logistic function
        - threshold: Classification threshold (default: 0.5)
    """
    
    def __init__(self):
        """Initialize the logistic regression node."""
        rospy.init_node('logistic_regression_node', anonymous=True)
        
        # Get parameters
        self.slope = rospy.get_param('~slope', 1.0)
        self.intercept = rospy.get_param('~intercept', 0.0)
        self.threshold = rospy.get_param('~threshold', 0.5)
        
        # Publishers
        self.result_pub = rospy.Publisher('/classification_result', String, queue_size=10)
        self.prob_pub = rospy.Publisher('/probability', Float64, queue_size=10)
        
        # Subscriber
        self.feature_sub = rospy.Subscriber('/input_features', Float64MultiArray, self.feature_callback)
        
        rospy.loginfo("Logistic Regression Node initialized.")
        rospy.loginfo(f"Parameters: slope={self.slope}, intercept={self.intercept}, threshold={self.threshold}")
    
    def feature_callback(self, msg):
        """
        Callback for incoming feature vectors.
        
        Parameters:
        -----------
        msg : std_msgs/Float64MultiArray
            Input feature vector
        """
        # Get feature value (assuming single feature for simplicity)
        if len(msg.data) == 0:
            rospy.logwarn("Empty feature vector received.")
            return
        
        x = msg.data[0]  # Use first feature
        
        # Compute linear combination
        z = self.slope * x + self.intercept
        
        # Compute probability using sigmoid function
        probability = sigmoid_function(z)
        
        # Classify based on threshold
        predicted_class = 1 if probability >= self.threshold else 0
        
        # Publish probability
        prob_msg = Float64()
        prob_msg.data = probability
        self.prob_pub.publish(prob_msg)
        
        # Publish classification result
        result = {
            'input_feature': x,
            'linear_combination': z,
            'probability': float(probability),
            'predicted_class': predicted_class,
            'threshold': self.threshold
        }
        
        result_msg = String()
        result_msg.data = json.dumps(result)
        self.result_pub.publish(result_msg)
        
        rospy.loginfo(f"Input: {x:.3f}, P(y=1): {probability:.3f}, Class: {predicted_class}")
    
    def run(self):
        """Main loop."""
        rospy.loginfo("Logistic regression node started.")
        rospy.spin()


def main():
    """Main entry point."""
    try:
        node = LogisticRegressionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
