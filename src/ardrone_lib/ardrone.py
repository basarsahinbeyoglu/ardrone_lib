#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ARDrone:
    # init method. image_cb is camera callback function
    def __init__(self, robotName):
        
        self.robotName = robotName
        self.is_shutdown = False
        
        self.isFlying = False
        self.isPosctrl = False
        self.isVelMode = False
        self.empty = Empty()
        self.bool = Bool()
        self.command = Twist()

        rospy.init_node(self.robotName)
        self.takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=2)
        self._land = rospy.Publisher('/drone/land', Empty, queue_size=2)
        self.reset = rospy.Publisher('/drone/reset', Empty, queue_size=2)
        self.posctrl = rospy.Publisher('/drone/posctrl', Bool, queue_size=2)
        self.cmd = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=10)
        self.velmode = rospy.Publisher('/drone/vel_mode', Bool, queue_size=2)

    # shutdown handler
    def shutdown():
        self.is_shutdown = True
        self.cmd.publish(Twist())
        rospy.loginfo('shutdown robot')

    def takeOff(self):
        if self.isFlying == True:
            return False
        self.takeoff.publish(self.empty)
        self.isFlying = True
        return True

    def land(self):
        if self.isFlying == False:
            return False
        self._land.publish(self.empty)
        self.isFlying = False
        return True

    def hover(self):
        if self.isFlying == False:
            return False
        
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

        self.cmd.publish(self.command)
        rospy.loginfo("Hovering...")
        return True

    def posCTRL(self,on):
        if self.isFlying == False:
            return False
        self.isPosctrl = on
        self.bool.data = on
        self.posctrl.publish(self.bool)
        if on:
            rospy.loginfo("Switching Position Control ON")
        else:
            rospy.loginfo("Switching Position Control OFF")
        return True

    def move(self,x,y,z,r,p,yw):
        if self.isFlying == False:
            return False
        
        self.command.linear.x = x
        self.command.linear.y = y
        self.command.linear.z = z
        self.command.angular.x = r
        self.command.angular.y = p
        self.command.angular.z = yw

        self.cmd.publish(self.command)
        rospy.loginfo("Hovering...")
        return True        

            
    def moveTo(self,x,y,z):
        if self.isFlying == False:
            return False
        if self.isPosctrl == False:
            return False
        
        self.command.linear.x = x
        self.command.linear.y = y
        self.command.linear.z = z
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

        self.cmd.publish(self.command)
        rospy.loginfo("Moving To Position...")
        return True