#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
#from nav_msgs.msg import Odometry
from math import floor

import numpy as np


# Import your custom code to implement your algorithm logic here
# for example:

class Map:
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize
        self.ysize = ysize
        self.grid_size = grid_size
        self.log_odds_map = np.zeros((self.xsize, self.ysize))

    def update_cell(self, x, y, value):
        self.log_odds_map[x, y] += value


class DemoNode:
    def __init__(self):

        # Initialize some necessary variables here
        self.node_frequency = None
        self.sub_scan = None

        #initialize the map
        #each tile with 10cm
        #
        grid_size = 0.1 
        self.map = Map(int(30/grid_size), int(30/grid_size), grid_size)
        
        # Store the data received from a scan sensor
        self.scan_sensor = 0.0
        self.pose = [0.0, 0.0, 0.0]
        
        # Initialize the ROS node
        rospy.init_node('demo_node')
        rospy.loginfo_once('Demo Node has started')

        # Load parameters from the parameter server
        self.load_parameters()

        # Initialize the publishers and subscribers
        self.initialize_subscribers()
        
        # Initialize the timer with the corresponding interruption to work at a constant rate
        self.initialize_timer()

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
        self.sub_scan= rospy.Subscriber('/scan', LaserScan, self.callback_scan)

    #def initialize_publishers(self):
    #    """
    #    You should/can initialize the publishers for the results of your algorithm here.
    #    """
    #
    #    # Initialize the publisher to the topic '/output_topic'
    #    self.pub_demo_topic = rospy.Publisher('/output_topic', Odometry, queue_size=10)

    def initialize_timer(self):
        """
        Here we create a timer to trigger the callback function at a fixed rate.
        """
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback)
        self.h_timerActivate = True

    def timer_callback(self, timer):
        """Here you should invoke methods to perform the logic computations of your algorithm.
        Note, the timer object is not used here, but it is passed as an argument to the callback by default.
        This callback is called at a fixed rate as defined in the initialization of the timer.
        At the end of the calculations of your EKF, UKF, Particle Filer, or SLAM algorithm, you should publish the results to the corresponding topics.
        """

        # Do something here at a fixed rate
        rospy.loginfo('Timer callback called at: %s', rospy.get_time())


        # Perform some logic with the sensor data received
        for i in range(360):
            dist = self.scan_sensor.ranges[i]
            norm_dist = dist/self.map.grid_size
            if dist < self.scan_sensor.range_min:
                continue
            ray = 0.0
            ray_dir = self.pose[2] + self.scan_sensor.angle_increment * i
            if ray_dir > 2*np.pi:
                ray_dir -= 2*np.pi
            xa = self.pose[0]
            ya = self.pose[1]
            xf = xa + norm_dist*np.cos(ray_dir)
            yf = ya + norm_dist*np.sin(ray_dir)
            a = (yf-ya)/(xf-xa)
            b= yf/(a*xf)
            if np.cos(ray_dir) > 0: xi = 1
            else : xi = 0
            if np.sin(ray_dir) > 0: yi = 1
            else : yi = 0
            
            while floor(xa) != floor(xf) and floor(ya) != floor(yf):
                #aplicar o InverseSensorModel()

                #intercepcao linha vertical
                xiv = floor(xa) + xi
                yiv = a*xiv+b
                distv = np.sqrt((yiv-ya)**2 + (xiv-xa)**2)
                #intercepcao linha horizontal
                yih = floor(xa) + yi
                xih = (yih-b)/a
                disth = np.sqrt((yih-ya)**2 + (xih-xa)**2)
                #decidir qual a celula seguinte
                if distv > disth:
                    xa = xih
                    ya = yih
                else:
                    xa = xiv
                    ya = yiv

                #excecao se for a 1 vez e se o sentido for negativo
                if xi == 0: xi = -1
                if yi == 0: yi = -1
                




    def callback_scan(self, msg):
        """
        Callback function for the subscriber of the topic '/demo_topic'. This function is called
        whenever a message is received by the subscriber. For example, you can receive the data from
        a sensor here and store it in a variable to be processed later or perform prediction steps
        of your algorithm here (it is up to you to decide).
        """

        # Do something with the message received
        rospy.loginfo('Received data: %s', msg.data)

        # Store the sensor message to be processed later (or process now depending on the application)
        self.scan_sensor = msg.data
        


def main():

    # Create an instance of the DemoNode class
    demo_node = DemoNode()

    # Spin to keep the script for exiting
    rospy.spin()

if __name__ == '__main__':
    main()