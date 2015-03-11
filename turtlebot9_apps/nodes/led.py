#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import kobuki_msgs.msg 


def talker():
    pub = rospy.Publisher('/mobile_base/commands/led1', kobuki_msgs.msg.Led)
    rospy.init_node('led')
    while not rospy.is_shutdown():
        msg=kobuki_msgs.msg.Led()
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
