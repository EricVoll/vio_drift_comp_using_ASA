#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
import message_filters

class Synchronizer:

    def __init__(self):
        rospy.init_node('topic_synchronizer', anonymous=True)

        # check arguments
        self.topic_base = rospy.get_param("~topic_base", "camera")
        self.camera_info_topic = rospy.get_param("~camera_info", "camera_info")
        self.camera_image_topic = rospy.get_param("~image", "raw_image")
        
        image_sub = message_filters.Subscriber(self.topic_base + self.camera_image_topic, Image)
        info_sub = message_filters.Subscriber(self.topic_base + self.camera_info_topic, CameraInfo)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.5)
        ts.registerCallback(self.callback)

        self.image_pub = rospy.Publisher(self.topic_base + "synched/" + self.camera_image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(self.topic_base + "synched/" + self.camera_info_topic, CameraInfo, queue_size=1)

        self.count = 0

    def callback(self, image, camera_info):
            
        self.count += 1

        if self.count % 2 == 0:
            return

        # print(image.header.stamp)
        # Solve all of perception here...
        camera_info.header.stamp = image.header.stamp
        self.image_pub.publish(image)
        self.info_pub.publish(camera_info)
        
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    commander = Synchronizer()
    commander.spin()