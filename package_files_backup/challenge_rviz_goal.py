#!/usr/bin/env python


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import sys
import math
import rospy
import tf
import collections
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Twist


class Turtlebot():

    def __init__(self):
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.reset_goals()
        # Subscriber for rviz coordinates
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.coordinates)
        
    def coordinates(self, data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y
        rospy.loginfo("coordenadas rviz recibidas: x = %f y = %f", self.goal_x, self.goal_y)

    def reset_goals(self):
        self.goal_x = np.Infinity
        self.goal_y = np.Infinity

    def command(self):
        rospy.loginfo("Command")
        if self.goal_x != np.Infinity or self.goal_y != np.Infinity:
            goal = PointStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time()
            
            goal.point.x = self.goal_x
            goal.point.y = self.goal_y
            goal.point.z = 0.0

            print("x = " + str(self.goal_x) + ", y = " + str(self.goal_y))
            goal.point.x = self.goal_x
            goal.point.y = self.goal_y
            goal.point.z = 0.0

            base_goal = PointStamped()
            try:
                base_goal = self.listener.transformPoint('base_footprint', goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return

            # TODO: put the control law here
            angular = 0.0
            linear = 0.0
            distance = math.sqrt((base_goal.point.x) ** 2 + (base_goal.point.y) ** 2)

            if distance > 0.1:
                angular = math.atan2(base_goal.point.y, base_goal.point.x) / 2.0
                if angular < 0.1 and angular > -0.1:
                    if distance > 0.5:
                        linear = 0.3
                    else:
                        linear = 0.1

            self.publish(linear, angular)
        else:
            self.reset_goals()
            return

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('robotcontrol', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        robot = Turtlebot()

        # What function to call when you ctrl + c
        rospy.on_shutdown(robot.shutdown)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            rospy.loginfo("Loop")
            # publish the velocity
            robot.command()
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrol node terminated.")
