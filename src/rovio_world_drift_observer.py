

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
from std_msgs.msg import String

from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class WorldDriftObserver:
    def __init__(self, penalty_seconds, drifted_callback):
        rospy.Subscriber("/rovio/T_G_W", TransformStamped, self.rovio_drifted, queue_size = 1)
        self.first_msg = True
        self.last_transform = None
        self.drifted_callback = drifted_callback
        self.debugPublisher = rospy.Publisher("drift/debug", String)
        self.penalty_seconds = penalty_seconds
        self.tolerance = 0.01
        self.lastDrift = 0 # rospy.Time.now().to_sec()


    def transformToNumpy(self, trans):
        t = trans.transform.translation
        r = trans.transform.rotation
        return np.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w])

    def rovio_drifted(self, transform):
        if self.first_msg:
            self.first_msg = False
            self.last_transform = self.transformToNumpy(transform)
            return
        
        arr = self.transformToNumpy(transform)
        diff = np.subtract(self.last_transform, arr)
        magn = np.linalg.norm(diff)
        
        if(magn > self.tolerance):
            self.lastDrift = rospy.get_time()
            self.drifted_callback()
        
        self.last_transform = arr

    # Returns true if the last drift happened within the tolerance
    def didRovioDrift(self):
        if(rospy.get_time() - self.lastDrift < self.penalty_seconds):
            return True
        else:
            return False
