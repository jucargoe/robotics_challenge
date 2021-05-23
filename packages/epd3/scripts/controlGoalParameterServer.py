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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import collections


class Turtlebot():
    goals = {}

    def __init__(self, goals):
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.goals = goals

        # Subscriber for rviz coordinates
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.coordinates)
        
    def coordinates(self, data):
        self.goals = [('goal_rviz', {
                    'x': data.pose.position.x,
                    'y': data.pose.position.y
                })]

    def command(self):
        rospy.loginfo("Command")
        goal = PointStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time()

        coordinates = self.goals[0][1]
        print("objective = " + self.goals[0][0]
              + ", x = " + str(coordinates['x'])
              + ", y = " + str(coordinates['y']))
        goal.point.x = coordinates['x']
        goal.point.y = coordinates['y']
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
        else:
            self.goals.pop(0)

        if len(self.goals) == 0:
            self.shutdown()

        self.publish(linear, angular)

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

        goals = {}
        # TODO 2: extend it to load more than one goal with base in "path" parameter
        if rospy.has_param('/coordinates'):
            coordinate_x = rospy.get_param('/coordinates/goal/x')
            coordinate_y = rospy.get_param('/coordinates/goal/y')
            if coordinate_x != 'None' and coordinate_y != 'None':
                goals = [('goal_args', {
                    'x': coordinate_x,
                    'y': coordinate_y
                })]
            elif rospy.has_param('/routes'):
                goals = sorted(rospy.get_param('/routes/paths_yml').items())

        # TODO 1: Load more internal parameters such as maximum speed, goal tolerance....

        print(' Goals to reach: ', goals)

        robot = Turtlebot(goals)

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
