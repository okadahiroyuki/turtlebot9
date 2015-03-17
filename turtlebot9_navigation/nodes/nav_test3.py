#!/usr/bin/env python
import rospy
import actionlib
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from geometry_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        self.tf_listener.waitForTransform(self.odom_frame,'/base_footprint', rospy.Time(), rospy.Duration(1.0))

        self.base_frame = '/base_footprint'
        self.tf_listener.waitForTransform(self.odom_frame,'/base_footprint', rospy.Time(), rospy.Duration(1.0))

        self.base_frame = '/base_link'





        # Are we running in the fake simulator?
#        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        #initial_pose = PoseWithCovarianceStamped()
        
        # Get the initial pose from the user
        #rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

#        initial_pose = PoseWithCovarianceStamped()
 #	initial_pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.000, 0.000, 0.0, 1.0))
 #       rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
        #rospy.init_node('initial_pose'); #, log_level=roslib.msg.Log.INFO)
        rospy.loginfo("Setting Pose")

        p   = PoseWithCovarianceStamped();
        msg = PoseWithCovariance();
        msg.pose = Pose(Point(0.0, -1.0, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0));
        msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942];
        p.pose = msg;
        pub.publish(p);

        
        # Make sure we have the initial pose
        #while initial_pose.header.stamp == "":
        #    rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # Begin the main loop and run through a sequence of locations
#        while not rospy.is_shutdown():
                        
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = Pose(Point(1.0, 0.0, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0))
 
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
            
            
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
            
        # Allow 5 minutes to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                rospy.loginfo("State:" + str(state))
            else:
              rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
        self.get_odom()
          
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def gen_pose():
      pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
      rospy.init_node('initial_pose'); #, log_level=roslib.msg.Log.INFO)
      rospy.loginfo("Setting Pose")

      p   = PoseWithCovarianceStamped();
      msg = PoseWithCovariance();
      msg.pose = Pose(Point(-0.767, -0.953, 0.000), Quaternion(0.000, 0.000, -0.0149, 0.9999));


    def get_odom(self):
      try:
        (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame,
self.base_frame, rospy.Time(0))
      except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

      print trans
      return


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
      
if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
