#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointPosition
from geometry_msgs.msg import PoseStamped

#
# def talker():
#     pub = rospy.Publisher('testtopic', JointPosition, queue_size=10)
#     rate = rospy.Rate(0.5) # 0.5hz
#     while not rospy.is_shutdown():
#         postion = calibrate_point()
#         rospy.loginfo(postion)
#         pub.publish(postion)
#         rate.sleep()

def calibrate_point(start):
    print('recevied')
    rospy.loginfo('start position', start)

def calibrate():
    print('running')
    rospy.init_node('calibrate', anonymous=True)
    if calibrate_point!=1:
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, calibrate_point)
        rospy.spin()

if __name__ == '__main__':
    calibrate()