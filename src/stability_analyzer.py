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





class StabilityAnalyzer:


    def __init__(self):

        rospy.init_node('stability_analyzer', anonymous=True)
        # self.anchor_id = "348d2057-4984-4eff-8dad-4deff362574b"
        self.anchor_id = "238f1d3c-f2df-4195-85d2-baa8067cbf63"
        
        # check arguments


        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()
        
        # All publishers
        self.info_publisher = rospy.Publisher("drift/debug", String, queue_size = 1)
        
        # All subscribers
        rospy.Subscriber('/fox/vrpn_client/estimated_transform', TransformStamped, self.groundtruth_received, queue_size=1)

        
        rospy.loginfo("Initialized anchor_manager")   

        self.start_time = 0
        self.start_asa_after = 5
        self.wait_time = 10
        self.started_asa = False
        self.asa_init = False
        self.asa = None
        self.request_asa_start = False
        self.current_level = 1
        self.error = 0.0
        self.count = 0
        self.error_speed = 0


        self.delta_time = 0
        self.last_frame_time = 0
        self.query_times = []
        self.last_query_time = 0
        self.data = []

        self.transformBuffer = None

        rospy.on_shutdown(self.save_data_to_disk)
    

    def spin(self):
        
        while not rospy.is_shutdown():

            sleeptime = 0.5
            
            if self.request_asa_start == True:
                self.request_asa_start = False
                self.asa_init = True
                self.asa = asa_handler(True, "", self.notify_anchor_found, self.report_last_anchor_id, 0)
                self.asa.SetIsHoloLensAnchor(True)

            time.sleep(sleeptime)

    def createCameraFrameFromIMU(self, transform):
        irc = TransformStamped()
        irc.header.stamp = transform.header.stamp
        irc.header.frame_id = transform.child_frame_id
        irc.child_frame_id = "camera0"
        irc.transform.translation.x = 0.0371982445732
        irc.transform.translation.y = -0.0397392343472
        irc.transform.translation.z = 0.0265641652917
        irc.transform.rotation.x = 0.00424533187421
        irc.transform.rotation.y = -0.0024078414729
        irc.transform.rotation.z = -0.711402164283  
        irc.transform.rotation.w = 0.702768197993  
        return irc


    def createScewedTransform_LinVel(self, transform):
        return transform
        # current_level in m/s interpreted

        # speed = dx / dt
        # dx = speed * dt 
        if self.error_speed != self.current_level * 2 * 0.1:
            self.error_speed = self.current_level * 2 * 0.1
            print("Level %i and error speed is: %f" % (self.current_level, self.error_speed))
        self.error = self.error + self.error_speed * self.delta_time
        transform.transform.translation.x = transform.transform.translation.x + self.error
        return transform


    def createScewedTransform_Rot(self, transform):
        r = transform.transform.rotation
        (roll, pitch, yaw)= euler_from_quaternion([r.x, r.y, r.z, r.w])

        if self.error_speed != self.current_level * 2 * 0.1:
            self.error_speed = self.current_level * 2 * 0.1
            print("Level %i and error speed is: %f" % (self.current_level, self.error_speed))
        
        print("Error speed: %f, delta time: %f " % (self.error_speed, self.delta_time))

        if(self.error > 100):
            self.error = 0

        self.error = self.error + self.error_speed * self.delta_time

        print("Yaw: %f + %f" %(yaw, self.error))

        yaw += self.error
        print(yaw)

        quat = quaternion_from_euler(roll, pitch, yaw)

        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        return transform

    def groundtruth_received(self, transform):

        self.delta_time = rospy.get_time() - self.last_frame_time

        self.count += 1
        if(self.start_time == 0):
            self.start_time = rospy.get_time()
        else:
            if self.asa_init == False and rospy.get_time() - self.start_time > self.start_asa_after:
                # dispatch on main thread
                self.last_query_time = rospy.get_time()
                self.request_asa_start = True

            if self.started_asa == False and rospy.get_time() - self.start_time > self.wait_time:
                # start asa
                self.started_asa = True
                print("Starting ASA!")
                self.asa.asa_request_anchor_find(self.anchor_id)

        transform.header.frame_id = "map"
        self.tf_broadcaster.sendTransform(transform)

        scewed = transform # self.createScewedTransform_LinVel(transform)
        scewed.child_frame_id = "imu"
        cam = self.createCameraFrameFromIMU(scewed)

        self.tf_broadcaster.sendTransform(scewed)
        self.tf_broadcaster.sendTransform(cam)

        self.last_frame_time = rospy.get_time()



    def transformToNumpy(self, trans):
        t = trans.transform.translation
        r = trans.transform.rotation
        (roll, pitch, yaw)= euler_from_quaternion([r.x, r.y, r.z, r.w])
        return numpy.array([t.x, t.y, t.z, roll, pitch, yaw])


##############################
# Callbacks from ASA handler #
##############################


    # the id of the last anchor that was found
    def report_last_anchor_id(self, anchor_id):
        pass

    # Called when an anchor is found an the transform relative to the configured world
    # coordiante system is passed as an arg
    def notify_anchor_found(self, transformStamped):
        print("FOUND THE ANCHOR!")
        self.query_times.append(rospy.get_time() - self.last_query_time)

        # log the anchor
        x = self.transformToNumpy(transformStamped)
        self.data.append(x)

        self.last_query_time = rospy.get_time()
        
        self.current_level += 1
        self.error = 0
        
        return

        if self.current_level % 100 == 0:
            self.asa.kill_asa_node()
            self.asa.start_asa_node()
        else:
            self.asa.request_asa_reset()

        print("Starting next level: %i" % self.current_level)
        self.asa.asa_request_anchor_find(self.anchor_id)

    def save_data_to_disk(self):
        ar = numpy.array(self.query_times)
        print(numpy.mean(ar))
        print(numpy.cov(ar, rowvar = False))
        numpy.savetxt("/home/eric/statistics_asa/query_times_drone.txt", ar, delimiter=" ")

        arr = numpy.array(self.data)
        numpy.savetxt("/home/eric/statistics_asa/anchors_drone.txt", arr, delimiter=" ")


if __name__ == '__main__':
    commander = StabilityAnalyzer()
    commander.spin()