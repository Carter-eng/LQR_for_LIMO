#!/usr/bin/env python3
# coding=UTF-8
import math
# import numpy as np
# from pylimo import limo
import rospy
# import re
# import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive


class CAV():
    def __init__(self, node_name):
        self.node_name = node_name
        self.position_z = 0
        self.position_x = 0
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.position_ip_z = 0
        self.position_ip_x = 0

        # construct publisher
        #rospy.init_node(self.node_name, anonymous=False)
        rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback,queue_size=1)
        self.pub = rospy.Publisher('vel_steer_'+self.node_name,AckermannDrive,queue_size=1) #topic name = CAV_Data
        rospy.Rate(10)

    def callback(self, msg):
        #Eul= self.quaternion_to_euler(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        self.position_z = msg.pose.position.z
        self.position_x = msg.pose.position.x
        x,y,z = self.quaternion_to_euler(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        self.position_yaw =  y
        self.Receivedata=1

    def steeringAngleToSteeringCommand(self,refAngle):
        x = refAngle
        y = 0.7*x
        return y

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

    def control(self,v_ref, steer_ref):
        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = self.steeringAngleToSteeringCommand(steer_ref)
        return drive_msg
