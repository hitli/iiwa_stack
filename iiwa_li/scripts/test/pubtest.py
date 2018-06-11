#!/usr/bin/env python
import rospy
from iiwa_msgs.msg import JointPosition
from geometry_msgs.msg import PoseStamped


def talker():
    joint_pub = rospy.Publisher('/iiwa/state/JointPosition', JointPosition, queue_size=30)
    pose_pub = rospy.Publisher('/iiwa/state/CartesianPose', PoseStamped, queue_size=30)
    rospy.init_node('fake_iiwa', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        joint_str = get_command_joint()
        rospy.loginfo(joint_str)
        joint_pub.publish(joint_str)

        pose_str = get_command_pose()
        rospy.loginfo(pose_str)
        pose_pub.publish(pose_str)

        rate.sleep()

def get_command_joint():
    command_point=JointPosition()
    command_point.position.a1 = 0
    command_point.position.a2 = 1
    command_point.position.a3 = 2
    command_point.position.a4 = 3
    command_point.position.a5 = 4
    command_point.position.a6 = 5
    command_point.position.a7 = 6
    return command_point

def get_command_pose():
    command_point = PoseStamped()
    command_point.pose.position.x = 0.121322486525
    command_point.pose.position.y = -0.4103412936
    command_point.pose.position.z = 0.776154925127
    command_point.pose.orientation.x = 0.795024514198
    command_point.pose.orientation.y = -0.57501980961
    command_point.pose.orientation.z = 0.0924391349392
    command_point.pose.orientation.w = 0.169538422535
    return command_point

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass