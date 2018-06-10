#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from iiwa_msgs.msg import JointPosition

def talker():
    pub = rospy.Publisher('/iiwa/state/JointPosition', JointPosition, queue_size=30)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = get_command_joint()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass