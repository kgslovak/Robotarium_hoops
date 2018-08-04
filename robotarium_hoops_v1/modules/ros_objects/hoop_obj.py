import numpy as np
import rospy
import tf

#=====================================================================================================

class HoopObj():
    def __init__(self, i, track_stat):
        self.radius = 0.35

        if track_stat == True:
            # Tracking frame identifiers
            self.worldFrame = rospy.get_param("~worldFrame", "/world")
            self.frame = 'hoop%d' %i
            # Create a listener and wait for tranform between frames to become available
            self.listener = tf.TransformListener()
            self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
            # Obtain initial position and orientation        
            self.updatepos()
            #self.center[2] = -0.7
        else:
            # Offline setup:
            if i == 0:
                self.center = np.array([  0.8,  0.0, -0.6])
            if i == 1:
                self.center = np.array([  0.0,  0.0, -0.6])
            if i == 2:
                self.center = np.array([ -0.8,  0.0, -0.6])
        
        # Helper points:
        self.front = self.center + np.array([  0.1, 0.0, 0.0])
        self.rear  = self.center + np.array([ -0.1, 0.0, 0.0])
        self.avoid = self.center + np.array([  0.0, -(self.radius + 0.3), 0.0])


    def updatepos(self):
        # Check latest transform
        t     = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
        check = self.listener.canTransform(self.worldFrame, self.frame, t)
        if check == True:
            self.center, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
            self.orientation        = tf.transformations.euler_from_quaternion(quaternion)



