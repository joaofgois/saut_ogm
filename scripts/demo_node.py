#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
#from nav_msgs.msg import Odometry
from math import floor
import matplotlib.pyplot as plt


import numpy as np
#teste

# Import your custom code to implement your algorithm logic here
# for example:

class Map:
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize
        self.ysize = ysize
        self.grid_size = grid_size
        self.alpha_norm = grid_size/grid_size  #largura desejada / gridsize
        self.log_odds_map = np.zeros((self.xsize, self.ysize))
        self.l_free = np.log(0.1/0.9)
        self.l_occ = np.log(0.9/0.1)


    def update_cell(self, x, y, value):
        self.log_odds_map[x, y] += value


class OGM_Node:
    def __init__(self):

        # Initialize some necessary variables here
        self.node_frequency = None

        #initialize the map
        #each tile with 10cm
        #
        grid_size = 0.01
        self.map = Map(int(60/grid_size), int(60/grid_size), grid_size)
        
        # Store the data received from a scan sensor
        self.scan_sensor = LaserScan()
        self.scan_sensor.angle_increment = 2*np.pi/360
        self.scan_sensor.ranges = [0.0]*360

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
        rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_pose)

    def initialize_timer(self):
        """
        Here we create a timer to trigger the callback function at a fixed rate.
        """
        #self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback)
        #self.h_timerActivate = True

    def timer_callback(self):
        """Here you should invoke methods to perform the logic computations of your algorithm.
        Note, the timer object is not used here, but it is passed as an argument to the callback by default.
        This callback is called at a fixed rate as defined in the initialization of the timer.
        At the end of the calculations of your EKF, UKF, Particle Filer, or SLAM algorithm, you should publish the results to the corresponding topics.
        """

        # Do something here at a fixed rate
        rospy.loginfo('Timer callback called at: %s', rospy.get_time())

        # Perform some logic with the sensor data received
        for i in range(360):

            scan_dist = self.scan_sensor.ranges[i]
            if scan_dist < self.scan_sensor.range_min:
                continue
            norm_dist = scan_dist/self.map.grid_size
            ray_dir = self.pose[2] + self.scan_sensor.angle_increment * i
            
            #if ray_dir > 2*np.pi:
            #    ray_dir -= 2*np.pi
            xa = self.pose[0]
            ya = self.pose[1]
            x_versor = np.cos(ray_dir)
            y_versor = np.sin(ray_dir)
            dist = 0
            xf = xa + (norm_dist + self.map.alpha_norm/2)*x_versor
            yf = ya + (norm_dist + self.map.alpha_norm/2)*y_versor

            if x_versor > 0: xi = 1
            else : xi = 0
            if y_versor > 0: yi = 1
            else : yi = 0

            tracing = True
            while tracing:
                x_prev = xa
                y_prev = ya
                dist_prev = dist
                #intercepcao linha vertical
                xiv = floor(xa)
                xiv += xi
                k_v = (xiv-xa)/x_versor
                yiv = k_v*y_versor + ya
                #intercepcao linha horizontal
                yih = floor(ya) + yi
                k_h = (yih-ya)/y_versor
                xih = k_h*x_versor + xa
                #decidir qual a celula seguinte
                if k_h < k_v:
                    #Intersecao Horizontal
                    xa = xih
                    ya = yih
                    dist = dist + k_h
                    if xi == -1: xi = 0
                    if yi == 0: yi = -1
                else:
                    #Intersecao Vertical
                    xa = xiv
                    ya = yiv
                    dist = dist + k_v
                    if yi == -1: yi = 0
                    if xi == 0: xi = -1

                x_med = (xa+x_prev)/2
                y_med = (ya+y_prev)/2

                #InvSenModel(norm_dist, dist, xa_prev, ya_prev)
                self.InvSenModel(dist_prev, dist, x_med, y_med, norm_dist)
                
                if (floor(xf) == floor(x_med) and floor(yf) == floor(y_med)) or dist_prev > 3.5/self.map.grid_size:
                    tracing = False

    def InvSenModel(self, dist_prev, dist, x_med, y_med, norm_dist):
        if dist < norm_dist-self.map.alpha_norm/2:
            #vazio
            self.map.log_odds_map[floor(x_med),floor(y_med)] += self.map.l_free
            return
        elif dist < norm_dist+self.map.alpha_norm/2 or dist_prev < norm_dist+self.map.alpha_norm/2:
            #dentro da parede
            self.map.log_odds_map[floor(x_med),floor(y_med)] += self.map.l_occ
            return
        else:
            return


    def callback_scan(self, msg):
        """
        Callback function for the subscriber of the topic '/demo_topic'. This function is called
        whenever a message is received by the subscriber. For example, you can receive the data from
        a sensor here and store it in a variable to be processed later or perform prediction steps
        of your algorithm here (it is up to you to decide).
        """

        # Do something with the message received
        #rospy.loginfo('Received data: %s', msg)

        # Store the sensor message to be processed later (or process now depending on the application)
        self.scan_sensor = msg

    def callback_pose(self,msg):         
         
        #rospy.loginfo('Received data: %s', msg)
        self.pose[0] = msg.pose.pose.position.x/self.map.grid_size + int(self.map.xsize/2)
        self.pose[1] = msg.pose.pose.position.y/self.map.grid_size + int(self.map.ysize/2) 
        self.pose[2] = 2*np.arcsin(msg.pose.pose.orientation.z)
        #self.pose.pose.position.x/y/z

        #-------------------------------
        self.timer_callback()


def main():

    # Create an instance of the DemoNode class
    node = OGM_Node()


    # Spin to keep the script for exiting
    rospy.spin()

    print(node.map.log_odds_map)
    plt.clf()
    plt.subplot(211)
    plt.imshow(node.map.log_odds_map, 'Greys') # log probabilities
    plt.subplot(212)
    plt.imshow(1.0 - 1./(1.+np.exp(node.map.log_odds_map)), 'Greys') #Probability
    plt.show()

if __name__ == '__main__':
    main()