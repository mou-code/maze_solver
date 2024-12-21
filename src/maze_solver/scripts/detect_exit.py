#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os
import time

# Maze boundaries
MAZE_BOUNDS = {
    "x_min": -10.0,
    "x_max": 10.0,
    "y_min": -10.0,
    "y_max": 10.0
}

class ExitDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('exit_detector', anonymous=True)

        # Initialize the velocity publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to the /odom topic
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Record start time
        self.start_time = rospy.Time.now()

        # Initialize variables
        self.last_position = None
        self.total_distance = 0.0
        self.last_move_time = rospy.Time.now()
        self.exploration_ended = False
        self.stuck_time_threshold = rospy.Duration(5)  # 5 seconds
        self.stuck_time = rospy.Duration(0)  # Initial stuck time

        rospy.loginfo("Exit detector initialized. Monitoring robot's position...")

    def odom_callback(self, msg):
        # Get the robot's current position
        position = msg.pose.pose.position
        x, y = position.x, position.y

        # Print the current position
        # rospy.loginfo("Current position: (x: %.2f, y: %.2f)", x, y)

        # Check if the robot has exited the maze bounds
        if x < MAZE_BOUNDS["x_min"] or x > MAZE_BOUNDS["x_max"] or y < MAZE_BOUNDS["y_min"] or y > MAZE_BOUNDS["y_max"]:
            rospy.logwarn("Exit detected at position: (%.2f, %.2f)", x, y)

            # Calculate time taken
            end_time = rospy.Time.now()
            time_taken = (end_time - self.start_time).to_sec()

            rospy.logwarn("Time taken to solve the maze: %.2f seconds", time_taken)
            rospy.logwarn("Total distance traveled: %.2f meters", self.total_distance)

            # Stop the robot immediately
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            rospy.logwarn("Robot stopped.")

            # Terminate the explore_lite node
            try:
                rospy.logwarn("Terminating explore_lite node...")
                os.system("rosnode kill /explore")
            except Exception as e:
                rospy.logerr("Failed to terminate explore_lite: %s", str(e))

            # Terminate the move_base node
            try:
                rospy.logwarn("Terminating move_base node...")
                os.system("rosnode kill /move_base")
            except Exception as e:
                rospy.logerr("Failed to terminate move_base: %s", str(e))

            try:
                rospy.logwarn("Terminating scan node...")
                os.system("rosnode kill /scan")
            except Exception as e:
                rospy.logerr("Failed to terminate scan: %s", str(e))

            try:
                rospy.logwarn("Terminating gmapping node...")
                os.system("rosnode kill /turtlebot3_slam_gmapping")
            except Exception as e:
                rospy.logerr("Failed to stop gmapping node: %s", str(e))

            try:
                rospy.logwarn("Terminating state publisher node...")
                os.system("rosnode kill /robot_state_publisher")
            except Exception as e:
                rospy.logerr("Failed to stop state publisher node: %s", str(e))
          # Stop the robot immediately
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            # Shutdown this node completely
            rospy.signal_shutdown("Exit detected! All nodes terminated.")
        
        # Handle case when the exploration is over, but the robot is still inside the bounds
        if not self.exploration_ended and (MAZE_BOUNDS["x_min"] <= x <= MAZE_BOUNDS["x_max"]) and (MAZE_BOUNDS["y_min"] <= y <= MAZE_BOUNDS["y_max"]):
            rospy.logwarn("Exploration ended, robot still inside bounds. Restarting explore_lite node...")

            # Restart the explore_lite node
            try:
                rospy.logwarn("Launching explore_lite node again...")
                os.system("roslaunch turtlebot3_explore_lite explore_lite.launch")
                self.exploration_ended = True  # Set exploration ended flag to true
            except Exception as e:
                rospy.logerr("Failed to launch explore_lite node: %s", str(e))

        # Check if the robot is stuck (no significant movement)
        self.check_if_stuck(position)

        # Calculate distance traveled
        if self.last_position is not None:
            distance = math.sqrt((x - self.last_position[0])**2 + (y - self.last_position[1])**2)
            self.total_distance += distance

            # Update the last movement time if the robot has moved significantly
            if distance > 0.05:  # Threshold for significant movement
                self.last_move_time = rospy.Time.now()
                self.stuck_time = rospy.Duration(0)  # Reset stuck time
                self.is_in_collision_recovery = False

        # Update last position
        self.last_position = (x, y)

    def check_if_stuck(self, position):
        # Check if the robot hasn't moved in the last 5 seconds
        if (rospy.Time.now() - self.last_move_time) > self.stuck_time_threshold:
            self.stuck_time += (rospy.Time.now() - self.last_move_time)
            rospy.logwarn("Robot appears to be stuck! Backing up...")

            # Back up the robot a little to recover from being stuck
            twist = Twist()
            twist.linear.x = -0.1  # Move backwards
            self.velocity_publisher.publish(twist)

            # Wait for 1 second before stopping
            rospy.sleep(1)

            # Stop the robot after moving back
            twist.linear.x = 0.0
            self.velocity_publisher.publish(twist)

            # Reset stuck time and continue
            self.last_move_time = rospy.Time.now()
            self.stuck_time = rospy.Duration(0)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ExitDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
