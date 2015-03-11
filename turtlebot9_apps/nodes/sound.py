#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import kobuki_msgs.msg 


def talker():
    pub = rospy.Publisher('/mobile_base/commands/sound', kobuki_msgs.msg.Sound)
    rospy.init_node('sound')
    while not rospy.is_shutdown():
        msg=kobuki_msgs.msg.Sound()
        msg.value = 1
        pub.publish(msg)
        rospy.sleep(2.0)
        msg.value = 0 
        pub.publish(msg)
        rospy.sleep(2.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
