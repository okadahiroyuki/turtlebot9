#!/usr/bin/env python
import rospy
from geometry_msgs.msg import * 
import time

class NavTest():
    def __init__(self):
        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
        rospy.init_node('nav', anonymous=True)


        p   = PoseWithCovarianceStamped();
        msg = PoseWithCovariance();
        msg.pose = Pose(Point(0.0, 0.0, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0));
        msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853];
        p.pose = msg;
        p.header.stamp = rospy.Time.now()
        p.header.frame_id="map"

        rospy.sleep(1.0)
        rospy.loginfo("Setting Pose")
        pub.publish(p);

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          r.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
      
if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
