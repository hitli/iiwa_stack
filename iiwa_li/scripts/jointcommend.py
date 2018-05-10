#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import quaternion_calculate as qc
from iiwa_msgs.msg import JointPosition


def talker():
    rospy.init_node('calibrate_commender', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=100)
    rospy.sleep(1)
    # commend_point=getcommand((-120,0,0,-90,-25,-90,0))  # 标定位置
    # commend_point = getcommand((-140, 30, 25, -80, -60, -80, -20))  # 扎蛋糕标定位置
    commend_point = qc.get_command_joint((-30, 0, 0, -90, -25, -90, 0))  # 0509师兄实验标定位置
    pub.publish(commend_point)


if __name__ == '__main__':
    talker()