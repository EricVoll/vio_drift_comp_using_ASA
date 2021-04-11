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

from rospy.numpy_msg import numpy_msg
import roslaunch


import time
import sys
import math

from statistics import DriftCompensator
from asa_handler import asa_handler
from rovio_world_drift_observer import WorldDriftObserver





class AnchorManager:


    def __init__(self):

        rospy.init_node('drift_compensator', anonymous=True)

        
        # check arguments

        self.rovio_ns = rospy.get_param("~rovio_ns", "")
        self.anchor_id_preset = rospy.get_param("~anchor_id_preset", "")
        self.compensate_position = rospy.get_param("~compensate_position", True)
        self.compensate_rotation = rospy.get_param("~compensate_rotation", False)
        self.covariance_mode = rospy.get_param("~covariance_mode", "empirical")
        self.manage_asa_node = rospy.get_param("~manage_asa_node", False)
        self.use_alternative_facts = rospy.get_param("~use_alternative_facts", False)
        self.use_interpolation = rospy.get_param("~interpolate_alt", False)
        self.send_update_mode = rospy.get_param("~send_update", 0)

        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()
        
        # All publishers
        self.odometry_publisher = rospy.Publisher(self.rovio_ns + '/odometry', Odometry, queue_size=1)
        self.alternativeOdomPublisher = rospy.Publisher(self.rovio_ns + '/odometry_alt', Odometry, queue_size=1)
        self.info_publisher = rospy.Publisher("drift/debug", String, queue_size = 1)

        if self.send_update_mode == 1:
            self.world_drift_observer = WorldDriftObserver(0.5, self.rovio_drifted)
        
        
        
        # All subscribers
        rospy.Subscriber('/rovio/odometry', Odometry, self.rovio_odometry_received, queue_size=1)

        self.world_frame = "world"  

        rospy.loginfo("Initialized anchor_manager")   

        self.stateUpdater = {}
        
        self.use_multiple_anchors = rospy.get_param("~multiple_anchors", False)
        self.mutliple_anchor_mode = rospy.get_param("~multiple_anchors_mode", 0) # 0: use the best, 1 use all
        self.did_update_with_anchor = {}
        self.block_updates_until_anchor = {}

        self.asa_handler = []
        asa_nodes_to_launch = 2 if self.use_multiple_anchors else 1
        self.asa_handler.append(asa_handler(self.manage_asa_node, self.anchor_id_preset, self.notify_anchor_found, self.report_last_anchor_id, 0))
        for i in range(1, asa_nodes_to_launch):
            self.asa_handler.append(asa_handler(self.manage_asa_node, "", self.notify_anchor_found, self.report_last_anchor_id, i))
            
        if self.anchor_id_preset != "":
            self.asa_handler[0].SetIsHoloLensAnchor(True)
        


    def spin(self):
        
        while not rospy.is_shutdown():

            sleeptime = 8
            rospy.loginfo("Sleeping for %i secs", sleeptime)
            time.sleep(sleeptime)


#########
# Utils #
#########

    def transformToNumpy(self, trans):
        t = trans.transform.translation
        r = trans.transform.rotation
        (roll, pitch, yaw)= euler_from_quaternion([r.x, r.y, r.z, r.w])
        return numpy.array([t.x, t.y, t.z, roll, pitch, yaw])

    def transformToHomogeneousTransformation(self, trans):
        t = trans.transform.translation
        r = trans.transform.rotation
        T_r = quaternion_matrix([r.x, r.y, r.z, r.w])
        T_t = translation_matrix([t.x, t.y, t.z])
        return numpy.dot(T_t, T_r)

    def stampToTime(self, stamp):
        t = rospy.Time(stamp.secs, stamp.nsecs)
        return t

    def transformToOdometry(self, transform, covariance):
        odom = Odometry()
        odom.header = transform.header
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.position.z = transform.transform.translation.z
        odom.pose.pose.orientation.x = transform.transform.rotation.x
        odom.pose.pose.orientation.y = transform.transform.rotation.y
        odom.pose.pose.orientation.z = transform.transform.rotation.z
        odom.pose.pose.orientation.w = transform.transform.rotation.w
        odom.pose.covariance = covariance
        return odom

    def debugPublish(self, text):
        self.info_publisher.publish(text)

#####################
# Rovio Subscribers #
#####################

    def rovio_drifted(self):
        # invalidate all anchors
        for key in self.block_updates_until_anchor.keys():
            self.block_updates_until_anchor[key] = True

        # self.debugPublish("Rovio Drifted!")

    def rovio_odometry_received(self, odom):
        for updater in self.stateUpdater.values():
            # report the covariance to all state updaters out there
            updater.ReportLatestStateCovariance(odom.pose.covariance)
        
        time = rospy.Time(odom.header.stamp.secs, odom.header.stamp.nsecs)

        self.send_update(time)

##############################
# Callbacks from ASA handler #
##############################


    # the id of the last anchor that was found
    def report_last_anchor_id(self, anchor_id):
        pass
        # self.debugPublish(anchor_id)

    # Called when an anchor is found an the transform relative to the configured world
    # coordiante system is passed as an arg
    def notify_anchor_found(self, transformStamped):
        anchor_id = transformStamped.child_frame_id
        
        if anchor_id not in self.stateUpdater.keys():
            self.stateUpdater[anchor_id] = DriftCompensator(rospy.get_param("~nr_anchor_instances", 5))

        # lookup tf and add the position into its dictionary entry
        x = self.transformToNumpy(transformStamped)
        T_wtA = self.transformToHomogeneousTransformation(transformStamped)

        self.stateUpdater[anchor_id].AddAnchorObservation(x)
        self.stateUpdater[anchor_id].ReportAnchorFound(T_wtA)
        
        # Allow updates being sent to rovio if blocked by mode
        self.did_update_with_anchor[anchor_id] = False
        self.block_updates_until_anchor[anchor_id] = False


#####################
# Rovio Poseupdates #
#####################

    def send_update(self, time):
        # GetLastTransform and publish the alternative fact
        found_anchor = False

        available_anchors = self.getAnchorsOrdered()

        if len(available_anchors) == 0:
            self.sendUpdateUsingAnchorId(None)
            return

        if self.mutliple_anchor_mode == 0:
            # only use the best anchor in this mode (the list of tuples is sorted)
            available_anchors = [available_anchors[0]]

        for tup in available_anchors:
            
            anchor_id = tup[0]

            if anchor_id in self.block_updates_until_anchor and self.block_updates_until_anchor[anchor_id]:
                # self.debugPublish("Skipped update. Rovio drifted. Waiting for anchor...")
                return
            
            self.sendUpdateUsingAnchorId(anchor_id)

    # Returns the anchors ordered with the best one first
    # Following this structure: (anchor_id, max_cov, goodness_value, age [secs]) 
    def getAnchorsOrdered(self):
        # find the best anchor
        secs_now = rospy.get_time()

        anchors = []

        for asa in self.asa_handler:
            
            (secs, anchor_id) = asa.getLastFoundTimeAndAnchorId()

            if secs == 0:
                # asa handler has not yet queried this anchor
                continue

            anchor_cov = max(self.stateUpdater[anchor_id].GetCovarianceEstimate().flatten().tolist())

            age = secs_now - secs

            goodness_value = 1/anchor_cov + 0.5 * math.exp(-age) - math.exp(age-10)

            anchors.append((anchor_id, anchor_cov, goodness_value, age))
        
        # Sort the anchors by their goodness value
        # Higher is better
        anchors.sort(key=lambda tup: tup[2], reverse=True)

        # print([x[2] for x in anchors])

        return anchors

    # Sends an update as configured using the specified anchor_id
    def sendUpdateUsingAnchorId(self, anchor_id):

        if self.send_update_mode == 2 and anchor_id != None and self.did_update_with_anchor[anchor_id]:
            return


        found_anchor = False
        if anchor_id != None:
            sourceFrame = anchor_id
            found_anchor = True
        else:
            sourceFrame = "map"

        try:
            alternative_drone = self.tf_Buffer.lookup_transform(sourceFrame, "imu", rospy.Time(0), rospy.Duration(0.03))
        except Exception:
            self.debugPublish("Fricked up the anchors " + sourceFrame)
            return

        # Change source frame to world_frame and send it out
        alternative_drone.header.frame_id = "map"
        alternative_drone.child_frame_id = "imu_alt"
        self.tf_broadcaster.sendTransform(alternative_drone)

        # self.debugPublish(str(found_anchor) + str(self.send_update_mode))

        if found_anchor and self.send_update_mode > 0:

            # if self.send_update_mode == 2 and self.did_update_with_anchor[anchor_id] == True:
            #     # This anchor was already used. Don't use it again if the node is these two modes.
            #     self.debugPublish("skipped " + anchor_id)
            #     return

            self.did_update_with_anchor[anchor_id] = True
            fetch_time = alternative_drone.header.stamp
            alternative_drone.header.stamp = rospy.Time.now()
            # self.debugPublish("Delta: %f " % (alternative_drone.header.stamp.to_sec() - fetch_time.to_sec()))

            # publish a tf (just for debugging purposes in rviz)
            alternative_drone.child_frame_id = "imu_update"
            self.tf_broadcaster.sendTransform(alternative_drone)

            # Publish the update to rovio either as a transform or odometry
            # The main difference is that the odometry includes covariance
            alternative_drone.child_frame_id = "imu"
            
            # use odometry with covariance
            cov = self.stateUpdater[anchor_id].GetCovarianceEstimate().flatten().tolist()
            alternative_odom = self.transformToOdometry(alternative_drone, cov)
            self.odometry_publisher.publish(alternative_odom)
            self.debugPublish("Max Cov is:  " + str(numpy.max(cov)))

    


if __name__ == '__main__':
    commander = AnchorManager()
    commander.spin()