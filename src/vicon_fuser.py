#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class ViconRovioFuser:
    def __init__(self):

        rospy.init_node('vicon_fuser', anonymous=True)
        
        # All publishers
        self.odometry_publisher = rospy.Publisher("/odometry", Odometry, queue_size=1)

        # All subscribers
        rospy.Subscriber('/fox/vrpn_client/estimated_odometry', Odometry, self.vicon_received, queue_size=1)

    def spin(self):
        rospy.spin()


#########
# Utils #
#########

    def vicon_received(self, odom):
        odom.header.frame_id = "map"
        odom.pose.covariance = np.diag([0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005]).flatten().tolist()
        self.odometry_publisher.publish(odom)


if __name__ == '__main__':
    commander = ViconRovioFuser()
    commander.spin()