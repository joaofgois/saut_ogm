#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
#from nav_msgs.msg import Odometry
from math import floor
import matplotlib.pyplot as plt


import numpy as np
#teste


#--------------------------------------------------------------------------------
# NOISE
# 68% of the observations lie within 1 standard deviation of the mean;
# 95% lie within two standard deviation of the mean;
# 99.9% lie within 3 standard deviations of the mean
SCAN_NOISE = 0.05
POSE_NOISE = 0.0
ANG_NOISE = 0.0
#--------------------------------------------------------------------------------

# Import your custom code to implement your algorithm logic here
# for example:

class Noiser_Node:
    def __init__(self):

        # Initialize some necessary variables here
        self.node_frequency = None
        self.pub_scan = None
        self.pub_pose = None
        self.scan_sensor = None
        self.position = None
        
        # Initialize the ROS node
        rospy.init_node('noiser_node')
        rospy.loginfo_once('Noiser Node has started')

        # Load parameters from the parameter server
        self.load_parameters()

        # Initialize the publishers and subscribers
        self.initialize_subscribers()
        self.initialize_publishers()

        
        # Initialize the timer with the corresponding interruption to work at a constant rate
        #self.initialize_timer()

    def initialize_publishers(self):

        # Initialize the publisher to the topic '/output_topic'
        self.pub_scan = rospy.Publisher('scan', LaserScan, queue_size=10)
        self.pub_pose = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size=10)


    def load_parameters(self):
        """
        Load the parameters from the configuration server (ROS)
        """

        # Node frequency of operation
        self.node_frequency = rospy.get_param('node_frequency', 30)
        rospy.loginfo('Node Frequency: %s', self.node_frequency)

    def initialize_subscribers(self):
        """
        Initialize the subscribers to the topics. You should subscribe to
        sensor data and odometry (if applicable) here
        """

        # Subscribe to the topic '/fake_sensor_topic'
        rospy.Subscriber('/scanner', LaserScan, self.callback_scan)
        rospy.Subscriber('/poser', PoseWithCovarianceStamped, self.callback_pose)

    def callback_scan(self, msg):

        # Store the sensor message to be processed later (or process now depending on the application)
        self.scan_sensor = msg

        scan_noise = abs(np.random.normal(1,SCAN_NOISE,360))
        self.scan_sensor.ranges *= scan_noise

        self.pub_scan.publish(self.scan_sensor)


    def callback_pose(self,msg):         

        pose_noise = np.random.normal(0,POSE_NOISE,2)

        self.position.pose.pose.position.x = self.position.pose.pose.position.x + pose_noise[0]
        self.position.pose.pose.position.y = self.position.pose.pose.position.y + pose_noise[1]
        self.position.pose.pose.orientation.z = 2*np.arcsin(msg.pose.pose.orientation.z)
        self.position.pose.pose.orientation.z = np.sin((np.arctan2(dir[1],dir[0]) + np.pi*np.random.normal(0,ANG_NOISE,1))/2)
        self.pub_pose.publish(self.position)



def main():
    # Create an instance of the DemoNode class
    node = Noiser_Node()


    # Spin to keep the script for exiting
    rospy.spin()


if __name__ == '__main__':
    main()