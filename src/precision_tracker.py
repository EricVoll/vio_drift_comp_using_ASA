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

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np

from rospy.numpy_msg import numpy_msg

from plot_data import PlotData
import os

import time
import sys


# Parameters:
# rovio_ns: The namespace of rovio. Used for publishing poses to rovio, Default ""
# run_name: The name used to store the results of this run.


class PerformanceTracker:
    def __init__(self):

        rospy.init_node('performance_tracker', anonymous=True)

        # check arguments

        self.rovio_ns = rospy.get_param("~rovio_ns", "")
        self.run_name = rospy.get_param("~run_name", "run")
        run_duration = rospy.get_param("~run_duration", 107)
        self.sampling_time = rospy.get_param("~sampling_time", 0.1)                               # The loop sleeps for this long
        min_buffer_size = run_duration / self.sampling_time    # Minimum columsn for arrays containing periodically fetched data
        self.start_time = 0

        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.world_frame = "odom"
        self.drone_frame = "imu"
        self.vicon_body_frame = "vicon/imu"

        # All subscribers
        # Subscribers listening to the asa ros wrapper
        rospy.Subscriber('/odometry', Odometry, self.odometry_update)
        rospy.Subscriber('/fox/vrpn_client/estimated_transform', TransformStamped, self.groundtruth_update, queue_size=1)
        self.rovio_odom_sub = rospy.Subscriber('/rovio/odometry', Odometry, self.trajestimate_update, queue_size=1)
        rospy.Subscriber('/odometry_alt', Odometry, self.alt_trajestimate_update, queue_size=1)
        self.use_alternative_facts = False


        rospy.on_shutdown(self.save_data_to_disk)

        self.stampedTrajEstimate = PlotData(min_buffer_size, self.sampling_time)
        self.stampedGroundtruth = PlotData(min_buffer_size, self.sampling_time)
        self.correctionData = PlotData(rospy.get_param("~plot_buffer_anchor_correction", 100))
        self.poseUpdateTimeStamps = []

        rospy.loginfo("Precision tracker intitialized with:")   
        rospy.loginfo("Duration of Rosbag reported: " + str(run_duration) + "s")   
        rospy.loginfo("Buffer size selected: " + str(min_buffer_size))   

    def spin(self):
        rospy.spin()


#########
# Utils #
#########

    def stampToSecs(self, stamp):
        t = rospy.Time(stamp.secs, stamp.nsecs)
        return t.to_sec()

#############################
# Recording external events #
#############################

    # Update from vicon
    def groundtruth_update(self, stampedTransform):
        time = self.stampToSecs(stampedTransform.header.stamp)

        # add to tf
        stampedTransform.header.frame_id = "map"
        self.tf_broadcaster.sendTransform(stampedTransform)
        pos = stampedTransform.transform.translation
        quat = stampedTransform.transform.rotation
        self.stampedGroundtruth.AddStampedDataPoint(time, [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w])

    # Trajectory update from rovio
    def trajestimate_update(self, odometry):
        if self.use_alternative_facts == False:
            self.submit_odometry_point(odometry)

    # Trajectory update from rovio
    def alt_trajestimate_update(self, odometry):
        # stop the normal subscriber since we are receiving alternative facts
        # self.rovio_odom_sub.unregsiter() unregistering is not working... wtf.

        # send our alternative facts to the normal tracking pipeline
        self.use_alternative_facts = True

        self.submit_odometry_point(odometry)

        # time = self.stampToSecs(odometry.header.stamp)
        # error = self.tf_Buffer.lookup_transform("fox", "imu_alt", rospy.Time(time), rospy.Duration(0.1)).transform.translation
        # error2 = self.tf_Buffer.lookup_transform("fox", "imu", rospy.Time(time), rospy.Duration(0.1)).transform.translation
        # magn = (error.x**2 + error.y**2 + error.z**2)**0.5
        # magn2 = (error2.x**2 + error2.y**2 + error2.z**2)**0.5
        
        
        # print("%.3f vs %.3f" % (magn, magn2))

    def submit_odometry_point(self, odometry):
        time = self.stampToSecs(odometry.header.stamp)

        pos = odometry.pose.pose.position
        quat = odometry.pose.pose.orientation
        self.stampedTrajEstimate.AddStampedDataPoint(time,  [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w])


    # Called whenever the drift compensator sends a correction signal
    def odometry_update(self, poseUpdateOdom):
        time = rospy.Time(poseUpdateOdom.header.stamp.secs, poseUpdateOdom.header.stamp.nsecs)
        msg_time = time.to_sec()
        
        try:
            trans = self.tf_Buffer.lookup_transform(self.world_frame, "imu", time, rospy.Duration(1.0))
            currentDroneVector = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        except:
            currentDroneVector = np.array([0,0,0])

        newPosUpdate = np.array([poseUpdateOdom.pose.pose.position.x, poseUpdateOdom.pose.pose.position.y, poseUpdateOdom.pose.pose.position.z])
        delta = np.linalg.norm(newPosUpdate - currentDroneVector)
        
        # Track the magnitude of the correction that had to be performed
        self.correctionData.AddDataPoint([msg_time, delta])
        # Track the timestmap of the pose update for the plots
        self.poseUpdateTimeStamps.append(msg_time)
        

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
        np.savetxt(path + "/correctionData.txt", self.correctionData.GetAsNumpy(), delimiter=" ")
        np.savetxt(path + "/poseUpdateTimeStamps.txt", np.array(self.poseUpdateTimeStamps), delimiter=" ")

if __name__ == '__main__':
    commander = PerformanceTracker()
    commander.spin()