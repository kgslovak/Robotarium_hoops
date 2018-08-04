import rospy
import tf

from std_msgs.msg      import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from math import sqrt

#=====================================================================================================

class CrazyflieObj():
    def __init__(self, i):
        # Frame names
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = 'crazyflie%d' %i
        # Create a listener and wait for tranform between frames to become available
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))

        # Initialize state
        self.state = 0
        # Initialize goal
        self.goal = PoseStamped()
        self.goal.header.seq   = 0
        self.goal.header.stamp = rospy.Time.now()

        # Topic names
        state_name    = '/crazyflie%d/CF_state' %i
        goal_name     = '/crazyflie%d/goal' %i
        cmd_vel_name  = '/crazyflie%d/cmd_vel' %i
        cmd_diff_name = '/crazyflie%d/cmd_diff' %i
        # Subscribers (callbacks used to obtain most recent data asynchronously)
        self.subState    = rospy.Subscriber(state_name,   String,      self.StateCallback)
        self.subGoal     = rospy.Subscriber(goal_name,    PoseStamped, self.GoalCallback)
        self.subCmd_vel  = rospy.Subscriber(cmd_vel_name, Twist,       self.CmdCallback)
        # Publishers        
        self.pubGoal     = rospy.Publisher(goal_name,     PoseStamped, queue_size=1)
        self.pubCmd_diff = rospy.Publisher(cmd_diff_name, Twist,       queue_size=1)

        # Obtain initial position and orientation        
        self.updatepos()
        # Publish first "command" message
        self.send_cmd_diff()


    def StateCallback(self, sdata):
        self.state = int(sdata.data)

    def GoalCallback(self, gdata):
        self.goal = gdata

    def CmdCallback(self, cdata):
        self.cmd_vel = cdata


    def updatepos(self):
        # Check latest transform
        t     = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
        check = self.listener.canTransform(self.worldFrame, self.frame, t)

        if check == True:
            self.position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
            self.orientation          = tf.transformations.euler_from_quaternion(quaternion)


    def goto(self, pnext):
        # Initialize the next goal to be published
        goal = PoseStamped()
        goal.header.frame_id = self.worldFrame
        goal.header.seq      = self.goal.header.seq + 1
        goal.header.stamp    = rospy.Time.now()

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

        goal.pose.position.x    = pnext[0]
        goal.pose.position.y    = pnext[1]
        goal.pose.position.z    = pnext[2]
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        self.pubGoal.publish(goal)


    def gotoT(self, pnext, s):
        # Initialize the next goal to be published
        goal = PoseStamped()
        goal.header.frame_id = self.worldFrame
        goal.header.seq      = self.goal.header.seq + 1
        goal.header.stamp    = rospy.Time.now()

        # Check latest transform
        t     = self.listener.getLatestCommonTime(self.worldFrame, self.frame)
        check = self.listener.canTransform(self.worldFrame, self.frame, t)

        if check == True:
            position, quaternion = self.listener.lookupTransform(self.worldFrame, self.frame, t)
            rpy = tf.transformations.euler_from_quaternion(quaternion)

            dx = pnext[0] - position[0]
            dy = pnext[1] - position[1]
            dz = pnext[2] - position[2]
            dq = 0 - rpy[2]

            quaternion = tf.transformations.quaternion_from_euler(0, 0, rpy[2] + s*dq)

            goal.pose.position.x    = position[0] + s*dx
            goal.pose.position.y    = position[1] + s*dy
            goal.pose.position.z    = position[2] + s*dz
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            self.pubGoal.publish(goal)

            error = sqrt(dx**2 + dy**2 + dz**2)
            if error < 0.1:
                return 1
            else:
                return 0

        else:
            return 0


    def send_cmd_diff(self, roll=0, pitch=0, yawrate=0, thrust=49000):
        # Theoretical default thrust for 35.89g is 39201 (actual is 49000)
        msg = Twist()
        msg.linear.x  = -pitch   # vx is -pitch (see crazyflie_server.cpp line 165)
        msg.linear.y  =  roll    # vy is  roll
        msg.linear.z  =  thrust
        msg.angular.z =  yawrate

        self.pubCmd_diff.publish(msg)



