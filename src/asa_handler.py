from asa_ros_msgs.msg import CreatedAnchor
from asa_ros_msgs.msg import FoundAnchor
from asa_ros_msgs.srv import FindAnchor, CreateAnchor
from threading import Thread

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
from rovio_world_drift_observer import WorldDriftObserver

import time
import sys


class ProcessListener(roslaunch.pmon.ProcessListener):

    def __init__(self, callback, objectReference):
        self.callback = callback
        self.objRef = objectReference

    def process_died(self, name, exit_code):
        self.callback(self.objRef)


class asa_handler:

    def __init__(self, manage_asa_node, anchor_id_preset, anchor_found_callback, last_anchor_id_callback, index = "", drift_free_frame = "map"):
        # self.world_drift_observer = WorldDriftObserver(0.5, self.rovio_drifted)
        rospy.loginfo("ASA Handler start " + 40*"=")
        self.manage_asa_node = manage_asa_node
        self.anchor_id_preset = anchor_id_preset
        self.drift_free_frame = drift_free_frame
        self.index = index
        self.reset_mode = 1 # 0 -> restart asa node, 1 -> request node reset
        self.auto_refind = True
        self.num_resets = 0

        if self.index != "":
            self.asa_node_name = "asa_ros_" + str(self.index)
        else:
            self.asa_node_name = "asa_ros"

        self.is_asa_running = False
        self.asa_find_anchor_service_name = self.asa_node_name +"/find_anchor"
        self.asa_reset_service_name = self.asa_node_name +"/reset"
        self.asa_create_anchor_service_name = self.asa_node_name +"/create_anchor"
        self.anchor_ids = []
        self.queryable_asa_id = ""
        self.last_found_secs = 0 # ros time stamp of the last time the anchor was found


        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        # Subscribers listening to the asa ros wrapper

        rospy.Subscriber(self.asa_node_name + '/found_anchor', FoundAnchor, self.asa_found_anchor_callback)
        rospy.Subscriber(self.asa_node_name + '/created_anchor', CreatedAnchor, self.asa_created_anchor_callback)
        self.asa_ready_subscriber = rospy.Subscriber(self.asa_node_name +'/operation_ready', EmptyMsg, self.asa_operation_ready)
        self.debugPublisher = rospy.Publisher("drift/debug", String, queue_size = 1)
        
        self.anchor_found_callback = anchor_found_callback
        self.last_anchor_id_callback = last_anchor_id_callback
        self.is_hololens_anchor = False

        if self.manage_asa_node:
            self.process_listener = ProcessListener(self.azure_node_died, self)
            self.is_asa_running = False
            self.start_asa_node()

        if self.anchor_id_preset != "":
            self.last_anchor_id_callback(self.anchor_id_preset)
            rospy.loginfo("Starting with anchor id preset " + self.anchor_id_preset)
            self.add_asa_frame(self.anchor_id_preset)
            self.asa_request_anchor_find(self.anchor_id_preset)
            self.queryable_asa_id = self.anchor_id_preset

        rospy.loginfo("ASA Handler initialized")

    def SetIsHoloLensAnchor(self, value):
        self.is_hololens_anchor = True

    def rovio_drifted(self):
        self.restart_asa()

    def azure_node_died(self, AnchorManager):
        self.debugPublisher.publish("ASA node died, restarting")
        self.debugPublisher.publish(AnchorManager.queryable_asa_id)
        AnchorManager.is_asa_running = False
        AnchorManager.start_asa_node()
        if AnchorManager.queryable_asa_id != "":
            # wait for asa node to launch
            time.sleep(3.5)
            AnchorManager.asa_request_anchor_find(AnchorManager.queryable_asa_id)

    def start_asa_node(self):

        if self.is_asa_running:
            return #we don't want to launch it twice.

        rospy.loginfo("------------------ launching asa_ros ------------------")

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)

        arg = ['mr-drone', 'asa_ros.launch']
        if self.anchor_id_preset != "":
            arg.append('anchor_id_preset:=' + self.anchor_id_preset)

        if self.index != "":
            arg.append('node_name:=' + self.asa_node_name)

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(arg)[0]
        roslaunch_args = arg[2:]
        
        self.asa_parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)], 
         verbose=False,
         force_screen=False,
         process_listeners = [self.process_listener])
        self.asa_parent.start()
        self.is_asa_running = True
        rospy.loginfo("------------------ launched asa_ros ------------------")
    
    def kill_asa_node(self):
        rospy.loginfo("------------------ killing asa_ros ------------------")
        if self.is_asa_running:
            self.asa_parent.shutdown()
            self.is_asa_running = False

    def asa_operation_ready(self, empty):

        rospy.loginfo("ASA is ready. Lets go!")

        if len(self.anchor_ids)>0:
            #We don't have to create multiple. This is only used once
            return
        
        # this is invoked when the asa ros wrapper reports to be ready for anchors
        # lets create one!
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "odom"
        # create an anchor at the origin of our frame. Does not really matter.
        # Just fix the quaternion
        t.transform.rotation.w = 1

        rospy.wait_for_service(self.asa_create_anchor_service_name)
        create_anchor = rospy.ServiceProxy(self.asa_create_anchor_service_name, CreateAnchor)
        create_anchor(t)

        # We only need this subscriber to kick off the whole process. We can remove it.
        self.asa_ready_subscriber.unregister()

    def add_asa_frame(self, anchor_id):
        if len(self.anchor_ids)==0:
            rospy.loginfo("Adding the first anchor " + anchor_id)
        
        if anchor_id not in self.anchor_ids:
            self.anchor_ids.append(anchor_id)

    
    # applies a constant rotation to the frame relative to odom
    def republish_frame_rotated(self, frame_name):

        rotatedFrameName = frame_name + "_static_rot"
        rotatedtrans = self.tf_Buffer.lookup_transform(self.drift_free_frame, frame_name, rospy.Time(0), rospy.Duration(0.1))
        rotation = rotatedtrans.transform.rotation

        quat1 = [-0.5, 0.5, 0.5, 0.5]
        quat2 = [rotation.x,rotation.y,rotation.z,rotation.w]
        quat3 = [ 0, 0, 0.7071068, 0.7071068 ]

        nr = quaternion_multiply(quat2, quat1)
        nr = quaternion_multiply(quat3, nr)

        rotatedtrans.transform.rotation.x = nr[0]
        rotatedtrans.transform.rotation.y = nr[1]
        rotatedtrans.transform.rotation.z = nr[2]
        rotatedtrans.transform.rotation.w = nr[3]
        rotatedtrans.child_frame_id = rotatedFrameName
        self.tf_static_broadcaster.sendTransform(rotatedtrans)

        return rotatedtrans

    # Adds the found anchors id to the available id list
    def asa_found_anchor_callback(self, data):

        anchor = data.anchor_in_world_frame
        branded_anchor_id = self.asa_node_name + data.anchor_id
        anchor.child_frame_id = branded_anchor_id
        self.tf_static_broadcaster.sendTransform(anchor) #created branded anchor

        # republish the anchor in map frame
        if self.is_hololens_anchor:
            static_anchor = self.republish_frame_rotated(branded_anchor_id)
        else:
            static_anchor = self.tf_Buffer.lookup_transform(self.drift_free_frame, branded_anchor_id, rospy.Time(0), rospy.Duration(0.1))
            static_anchor.child_frame_id = branded_anchor_id + "_static"
            self.tf_static_broadcaster.sendTransform(static_anchor)

        # call back into object owner
        self.anchor_found_callback(static_anchor)
        self.last_found_secs = rospy.get_time()

        anchor_id_to_report = static_anchor.child_frame_id
        self.add_asa_frame(anchor_id_to_report)
        self.last_anchor_id_callback(anchor_id_to_report)
        self.queryable_asa_id = data.anchor_id
        
        if self.manage_asa_node and self.auto_refind:
            # restart ASA session
            self.restart_asa()


    # Adds the created anchors id to the available id list if it succeeded
    def asa_created_anchor_callback(self, data):
        rospy.loginfo("Anchor created. Searching for it.")
        #ask asa to find the anchor so that it creates a tf and we don't have to handle that
        self.asa_request_anchor_find(data.anchor_id)

    def asa_request_anchor_find(self, anchor_id_to_find):
        if anchor_id_to_find not in self.anchor_ids:
            self.anchor_ids.append(anchor_id_to_find)

        rospy.wait_for_service(self.asa_find_anchor_service_name)
        find_anchor = rospy.ServiceProxy(self.asa_find_anchor_service_name, FindAnchor)
        find_anchor(anchor_id_to_find)

    def request_asa_reset(self):        
        rospy.wait_for_service(self.asa_reset_service_name)
        reset = rospy.ServiceProxy(self.asa_reset_service_name, EmptySrv)
        reset()

    def restart_asa(self):

        self.num_resets += 1

        if self.reset_mode == 0 or self.num_resets % 100 == 0:
            self.kill_asa_node()
            time.sleep(0.1)
            self.start_asa_node()
            time.sleep(3.5)
        else:
            if self.reset_mode == 1:
                self.request_asa_reset()

        self.asa_request_anchor_find(self.queryable_asa_id)
        
        thread = Thread(target = self.reset_if_stale, args = (8, self))
        thread.start()
    
    def reset_if_stale(self, seconds, asa_handler):
        time.sleep(seconds)
        if  rospy.get_time() - asa_handler.getLastFoundTimeAndAnchorId()[0] > seconds:
            asa_handler.request_asa_reset()
            asa_handler.debugPublisher.publish("Reset asa " + asa_handler.asa_node_name)

    def getLastFoundTimeAndAnchorId(self):
        if len(self.anchor_ids) > 0:
            return (self.last_found_secs, self.anchor_ids[-1])
        else:
            return (0, "")