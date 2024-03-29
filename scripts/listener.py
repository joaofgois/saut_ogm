#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped




def callback_chatter(data):
    rospy.loginfo(rospy.get_caller_id() + "New message: %s", data.data)

def callback_scan(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)

    rospy.Subscriber("chatter", String, callback_chatter)

    rospy.Subscriber("amcl_pose", LaserScan, callback_scan)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()