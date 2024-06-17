import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import socket
import time
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
import rospy
from LIMO_ROS_SETUP import CAV
import LIMO_LQR

def main():
    #initialize CAV, PID values, and another parameters
    isMain1 = False
    CAV1 = CAV("limo770")

    transmissionRate = 10
    dt = 1/transmissionRate # or 0.1
    rate = rospy.Rate(transmissionRate) # 1Hz
    v_ref_CAV1 = 0.5 # set between 0.5 and 0.6
    lqr = LIMO_LQR.LQR()
    speed = 0
    x1 = np.array([
        [.5],
        [1],
        [-.70],
        [0]
    ])
    x2 = np.array([
        [1.0],
        [2.0],
        [-1.57],
        [0]
    ])
    xs = [x1,x2]
    x_plot = []
    y_plot = []
    A,B = lqr.getAB(CAV1.position_yaw)
    #depending on if the car starts at main path or merging path, initialize different starting paths, points, and PID value
    # access the array that stores the distance of each line, then change velocity if the length is quite large
    # v_ref_CAV1 = 0.5 # set between 0.5 and 0.6, or higher if line is longer

    for xd in xs:
        for count in range(20,1,-1):

        #if the cav is near a critical point (which are turning corners), set current line, starting point, and PID values to be that of the next line

            x=np.array([
                [CAV1.position_z],
                [CAV1.position_x],
                [CAV1.position_yaw],
                [speed]
            ])
            # print(x,xd)
            x_plot.append(x[0,0])
            y_plot.append(x[1,0])
            if not -2<x[0,0] or not x[0,0]<2:
                now = time.time()
                drive_msg_CAV1 = CAV1.control(0,0)
                print(drive_msg_CAV1)
                CAV1.pub.publish(drive_msg_CAV1)
                break
            if not -2<x[1,0] or not x[1,0]<2:
                now = time.time()
                drive_msg_CAV1 = CAV1.control(0,0)
                print(drive_msg_CAV1)
                CAV1.pub.publish(drive_msg_CAV1)
                break
            if count%2 == 0:
                A,B = lqr.getAB(x[2,0])
                K = lqr.getK(x,xd,count,A,B)
            u = lqr.getControl(x,xd,K)
            # u = lqr.lqr(x,xd,count,A,B)
            ang = math.asin(np.clip(u[0,0]*lqr.L/(u[1,0]+x[3,0]),-1,1))
            drive_msg_CAV1 = CAV1.control(u[1,0]+x[3,0],ang)
            CAV1.pub.publish(drive_msg_CAV1)
            speed = u[1,0]+x[3,0]

            time.sleep(dt)


            #rospy.spin()

    drive_msg_CAV1 = CAV1.control(0,0)
    CAV1.pub.publish(drive_msg_CAV1)
    drive_msg_CAV1 = CAV1.control(0,0)
    CAV1.pub.publish(drive_msg_CAV1)

    #print(stop)
    # plt.plot(x_plot[1:],y_plot[1:], '-r')
    # plt.show()
if __name__ == '__main__':
    main()
