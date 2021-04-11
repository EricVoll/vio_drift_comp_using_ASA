#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from tf.transformations import *
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from asa_ros_msgs.msg import FoundAnchor

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np

from rospy.numpy_msg import numpy_msg

from plot_data import PlotData
import os

import time
import sys

class PerformanceTracker:
    def __init__(self):

        rospy.init_node('performance_tracker', anonymous=True)

        # check arguments

        self.run_name = rospy.get_param("~run_name", "run")
        run_duration = rospy.get_param("~run_duration", 107)
        self.sampling_time = rospy.get_param("~sampling_time", 0.04)     # The loop sleeps for this long
        min_buffer_size = run_duration / self.sampling_time             # Minimum columsn for arrays containing periodically fetched data
        self.start_time = 0

        self.groundtruth_frame = rospy.get_param("~groundtruth_frame", "fox")
        self.static_frame = rospy.get_param("~static_frame", "map")
        self.estimate_frame = rospy.get_param("~estimate_frame", "imu")
        self.groundtruth_topic = rospy.get_param("~groundtruth_topic", "")
        self.broadcast_groundtruth = rospy.get_param("~broadcast_groundtruth", False)
        self.trigger_topic = rospy.get_param("~trigger_topic", "/rovio/odometry")

        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        rospy.on_shutdown(self.save_data_to_disk)

        self.stampedTrajEstimate = PlotData(min_buffer_size, self.sampling_time)
        self.stampedGroundtruth = PlotData(min_buffer_size, self.sampling_time)
        self.anchors = []

        if self.broadcast_groundtruth:
            rospy.loginfo(self.groundtruth_topic)
            rospy.Subscriber(self.groundtruth_topic, TransformStamped, self.groundtruth_received, queue_size=1)

        rospy.Subscriber(self.trigger_topic, Odometry, self.trigger_est, queue_size=1)
        rospy.Subscriber(self.trigger_topic, Odometry, self.trigger_gro, queue_size=1)

        rospy.Subscriber("asa_ros/found_anchor", FoundAnchor, self.found_anchor,   queue_size=2)
        rospy.Subscriber("asa_ros_0/found_anchor", FoundAnchor, self.found_anchor, queue_size=2)
        rospy.Subscriber("asa_ros_1/found_anchor", FoundAnchor, self.found_anchor, queue_size=2)
        rospy.Subscriber("asa_ros_2/found_anchor", FoundAnchor, self.found_anchor, queue_size=2)
        rospy.Subscriber("asa_ros_3/found_anchor", FoundAnchor, self.found_anchor, queue_size=2)

        self.count = 0

        rospy.loginfo("Tracking as groundtruth: " + self.groundtruth_frame)
        rospy.loginfo("Tracking as estimate: " + self.estimate_frame)
        rospy.loginfo("Using : " + self.static_frame + " as a base")


        self.spin()

    def spin(self):
        rospy.spin()
        


    def stampToSecs(self, stamp):
        t = rospy.Time(stamp.secs, stamp.nsecs)
        return t.to_sec()
    def stampToTime(self, stamp):
        t = rospy.Time(stamp.secs, stamp.nsecs)
        return t

    def trigger_est(self, odom):
        time = self.stampToTime(odom.header.stamp)
        self.track_frame(self.estimate_frame, self.stampedTrajEstimate, time)

    def trigger_gro(self, odom):
        time = self.stampToTime(odom.header.stamp)
        self.track_frame(self.groundtruth_frame, self.stampedGroundtruth, time)

    # Query the frames and store their locations
    def track_frames(self, time):
        self.track_frame(self.estimate_frame, self.stampedTrajEstimate, time)
        self.track_frame(self.groundtruth_frame, self.stampedGroundtruth, time)
        
    def track_frame(self, frame_id, data_storage, time):
        
        try:
            tf = self.tf_Buffer.lookup_transform(self.static_frame, frame_id, time, rospy.Duration(0.01))
            
        except Exception as e:
            print("Exception while looking up tfs in precision tracker")
            print(e)
            return
        
        self.submit_transform_stamped(tf, data_storage, time)


    # Add the data point to the specified storage
    def submit_transform_stamped(self, transform, data_storage, time):
        time = time.to_sec()
        pos = transform.transform.translation
        quat = transform.transform.rotation
        data_storage.AddStampedDataPoint(time,  [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w])

    # publish groundtruth as tf
    def groundtruth_received(self, transform):
        transform.header.frame_id = self.static_frame
        self.tf_broadcaster.sendTransform(transform)

        return
        # we can republish a camera frame if we want to
        irc = TransformStamped()
        irc.header.stamp = transform.header.stamp
        irc.header.frame_id = "fox"
        irc.child_frame_id = "camera1"
        irc.transform.translation.x = 0.0371982445732
        irc.transform.translation.y = -0.0397392343472
        irc.transform.translation.z = 0.0265641652917
        irc.transform.rotation.x = 0.00424533187421
        irc.transform.rotation.y = -0.0024078414729
        irc.transform.rotation.z = -0.711402164283  
        irc.transform.rotation.w = 0.702768197993  
        self.tf_broadcaster.sendTransform(irc)
        

    def found_anchor(self, foundAnchorMsg):
        self.anchors.append(rospy.get_time())

#############################
# Wrapping up this test run #
#############################


    # Saves all collected matrices and plots to the disk
    def save_data_to_disk(self):
        # Update the plot a last time

        path = "/home/eric/statistics/"+self.run_name
        try:
            os.makedirs(path)
        except:
            rospy.loginfo("Folder already existed: " + path)
        np.savetxt(path + "/stamped_traj_estimate.txt", self.stampedTrajEstimate.GetAsNumpy(), delimiter=" ")
        np.savetxt(path + "/stamped_groundtruth.txt", self.stampedGroundtruth.GetAsNumpy(), delimiter=" ")
        np.savetxt(path + "/anchors.txt", np.array(self.anchors), delimiter = " ")

if __name__ == '__main__':
    commander = PerformanceTracker()
    commander.spin()