#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from iiwa_msgs.msg import JointPosition


def talker():
    rospy.init_node('calibrate_commender', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=100)
    rospy.sleep(1)
    # commend_point=getcommand((-120,0,0,-90,-25,-90,0))  # 标定位置
    commend_point = getcommand((-140, 30, 25, -80, -60, -80, -20))  # 扎蛋糕标定位置
    # commend_point = getcommand((0, 0, 0, -20, 0, 0, 0))
    pub.publish(commend_point)


def getcommand(calibrate_point):
    command_point=JointPosition()
    command_point.position.a1 = calibrate_point[0]/180.0*math.pi
    command_point.position.a2 = calibrate_point[1]/180.0*math.pi
    command_point.position.a3 = calibrate_point[2]/180.0*math.pi
    command_point.position.a4 = calibrate_point[3]/180.0*math.pi
    command_point.position.a5 = calibrate_point[4]/180.0*math.pi
    command_point.position.a6 = calibrate_point[5]/180.0*math.pi
    command_point.position.a7 = calibrate_point[6]/180.0*math.pi
    print (calibrate_point[3])
    print (calibrate_point[3] / 180.0)
    print (calibrate_point[3]/180.0*math.pi)
    print(command_point.position.a4)
    print 'calibrate_point',calibrate_point
    print 'command_point',command_point
    return command_point


if __name__ == '__main__':
    talker()