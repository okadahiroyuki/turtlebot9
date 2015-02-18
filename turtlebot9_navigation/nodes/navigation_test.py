#!/usr/bin/env python
mport rospy
import actionlib
import tf
import time
from actionlib_msgs.msg import *
from geometry_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class NavTest():
    def __init__(self):
        #ロボットの初期位置を設定するためのノード
        pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)

        rospy.init_node('navigation_test', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # パラメータの設定
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)

        #オドメトリを取得するための準備
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
        self.fake_test = rospy.get_param("~fake_test", False)

        # ゴールに到着したときに返す値
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()
        '''
        map134
        locations['home'] = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.000, 0.000, 0.0, 1.0))
        locations['book_shelf'] = Pose(Point(5.046, -0.928, 0.0), Quaternion(0.000, 0.000, -0.707, 0.707))
        locations['tv'] = Pose(Point(4.559, 1.719, 0.0), Quaternion(0.000, 0.000, 0.348, 0.937))
        locations['kitchen_table'] = Pose(Point(1.106, 1.886, 0.0), Quaternion(0.000, 0.000, 1.0, 0.0))
        locations['fridge'] = Pose(Point(0.586, -1.242, 0.0), Quaternion(0.000, 0.000, -0.707, 0.707))

        '''
        locations['home'] = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.000, 0.000, 0.0, 1.0))

        # 緊急停止用に速度をパブリッシュするため
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # action serverが有効になるまで60秒待つ
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # ロボットの初期位置を設定する
        set_pos(Pose(Point(0.0, 0.0, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0)))

        '''
        initial_pose = PoseWithCovarianceStamped()
        msg = PoseWithCovariance();
        msg.pose = Pose(Point(0.0, 0.0, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0));
        msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853];
        initial_pose.pose = msg;
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id="map"
        rospy.sleep(1.0)
        rospy.loginfo("Setting Pose")
        pub.publish(initial_pose);
        '''

        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            self.get_odom()

            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                sequence = sample(locations, n_locations)
                # Skip over first location if it is the same as
                # the last location
                if sequence[0] == last_location:
                    i = 1
            
            # Get the next location in the current sequence
            location = sequence[i]
                        
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
            # Increment the counters
            i += 1
            n_goals += 1
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))
            
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
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.sleep(self.rest_time)


    def set_pose(self, pos):
        # ロボットの初期位置を設定する
        p = PoseWithCovarianceStamped()
        msg = PoseWithCovariance();
        msg.pose = pos;
        msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853];
        p.pose = msg;
        p.header.stamp = rospy.Time.now()
        p.header.frame_id="map"
        rospy.sleep(1.0)
        rospy.loginfo("Setting Pose")
        pub.publish(p);

        
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

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
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
