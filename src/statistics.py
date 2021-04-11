from rospy.numpy_msg import numpy_msg
import numpy as np

class DriftCompensator:

    def __init__(self, buffer_size):
        vector_length = 6
        self.data = []
        self.buffer_size = buffer_size
        self.T_w0A = np.eye(4) # this will always stay the same (its where we create the anchor)
        self.T_wtA = np.eye(4) # This will be set to the value where the anchor is found

        self.current_covariance = np.zeros((6,6))
        self.cov_T_w0A = np.zeros((6,6)) # This will stay zero, because we positioned it there
        self.latest_covariance = [] # Save this as a normal array only, not as numpy array. We won't use most of them 

    # Adds an anchor observation to the storage limited in size by 
    # the buffer_size parameter
    def AddAnchorObservation(self, x):
        self.data.append(x)

        if(len(self.data)>self.buffer_size):
            self.data.pop(0)

    # Returns the last anchor
    def GetLast(self):
        return self.data[-1]

    def ReportLatestStateCovariance(self, matrix):
        self.latest_covariance = matrix

    # Returns the covariance matrix of all anchor observation data
    def GetCovarianceEstimate(self):
        if(len(self.data)<2):
            # We only received one or no item
            # Return some relatively high cov.
            return np.eye(6) * 0.1

        matrix_part = np.array(self.data)

        self.current_covariance = np.cov(matrix_part, rowvar = False)

        cov1 = np.add(self.cov_T_w0A, self.current_covariance)
        
        return np.add(cov1, np.array(self.latest_covariance).reshape((6,6)))

    # Stores the reported transformation from the world coordinate at time t to the anchor
    def ReportAnchorFound(self, T_wtA):
        self.T_wtA = T_wtA

    # Returns the transform from the current world w^t frame to the initial world frame w^0
    def GetStateDriftTransform(self):
        T_w0w1 = self.T_w0A * np.linalg.inv(self.T_wtA)
        return T_w0w1