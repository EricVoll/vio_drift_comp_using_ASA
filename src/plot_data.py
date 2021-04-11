
import numpy as np

class PlotData:

    def __init__(self, buffer_size, sampling_time = 0):
        self.data = []
        self.buffer_size = buffer_size
        self.count = 0
        self.sampling_time = sampling_time
        self.last_time = 0
    # Adds an anchor observation to the storage limited in size by 
    # the buffer_size parameter
    def AddDataPoint(self, x):
        self.data.append(x)
        self.count += 1

        if(len(self.data) > self.buffer_size):
            self.data.pop(0)

    # This method discards values x if the timestamp compared to the last accepted timestamp is younger than the sampling time
    def AddStampedDataPoint(self, timestamp, x):
        if timestamp - self.last_time > self.sampling_time:
            x.insert(0, timestamp)
            
            self.data.append(x)
            self.count += 1
            
            if(len(self.data) > self.buffer_size):
                self.data.pop(0)
        
            self.last_time = timestamp

    # Returns the last anchor
    def GetLast(self):
        if(len(self.data)>0):
            return self.data[len(self.data)-1]
        return [0,0,0]
    
    #returns the data as numpy arra
    def GetAsNumpy(self):
        return np.array(self.data)

    def NumberOfPointsAdded(self):
        return self.count