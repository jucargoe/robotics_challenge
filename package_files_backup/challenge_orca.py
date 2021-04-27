#!/usr/bin/env python

import sys
import math
import rospy
import tf
import numpy as np
# import pyorca
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Orca():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.robot_position_x = 0.0
        self.robot_position_y = 0.0
        self.goal_x = np.Infinity
        self.goal_y = np.Infinity
        self.angular = 0.0
        self.linear = 0.0

        rospy.Subscriber("/ScanDownsampler/laser", LaserScan, self.scan)
        rospy.Subscriber("/RvizListener/coordinates", PointStamped, self.coordinates)
        rospy.Subscriber('/odom', Odometry, self.callback)

    def callback(self, odometry):
        # print(odometry)
        self.robot_position_x = odometry.pose.pose.position.x
        self.robot_position_y = odometry.pose.pose.position.y
        # print(str(self.robot_position_x), str(self.robot_position_y))

    def scan(self, laser_data):
        print("scan")
        # position = (0., 0.)
        # velocity = (self.linear, self.angular)
        # radius = 1.
        # max_speed = 5
        # pref_velocity = 0
        # robot = pyorca.Agent(position, velocity, radius, max_speed, pref_velocity)
        # agents = []

        # for angle, laser_range in enumerate(laser_data.ranges):
        #     if laser_range != np.Infinity:
        #         x = np.sin(angle) * laser_range
        #         y = np.cos(angle) * laser_range
        #         print("x: " + str(x) + ", y: " + str(y))
        #         position = (x, y)
        #         velocity = (0., 0.)
        #         radius = 1.
        #         max_speed = 0
        #         pref_velocity = 0
        #         agent = pyorca.Agent(position, velocity, radius, max_speed, pref_velocity)
        #         agents.append(agent)
        #         # print(laser_range)
        
        # vel, lines = pyorca.orca(robot, agents, 0, 0.)
        # print("vel: " + str(vel) + ", lines: " + str(lines))

    def coordinates(self, coordinates):
        self.goal_x = coordinates.point.x
        self.goal_y = coordinates.point.y

    def command(self):
        #rospy.loginfo("Command")
        if self.goal_x != np.Infinity or self.goal_y != np.Infinity:
            #print("x = " + str(self.goal_x) + ", y = " + str(self.goal_y))
            distance = math.sqrt((self.goal_x) ** 2 + (self.goal_y) ** 2)

            # FIXME
            if distance > 0.1:
                self.angular = math.atan2(self.goal_y, self.goal_x) / 2.0
                if self.angular > 0.3:
                    self.angular = 0.3
                elif self.angular < -0.3:
                    self.angular = -0.3

                if self.angular < 0.1 and self.angular > -0.1:
                    self.linear = 0.1 if self.linear == 0.0 else self.linear * 1.15

                    if self.linear > 0.5:
                        self.linear = 0.5
                else:
                    self.linear /= 2
            else:
                self.angular = 0.0
                self.linear = 0.0

            # print("distance: " + str(distance) + ", angular: " + str(self.angular) + ", linear: " + str(self.linear))
            self.publish(self.linear, self.angular)

    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)

    def shutdown(self):
        rospy.loginfo("Stop Orca")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('Orca', anonymous=False)
        rospy.loginfo("To stop Orca CTRL + C")
        orca = Orca()
        rospy.on_shutdown(orca.shutdown)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #rospy.loginfo("Loop")
            orca.command()
            r.sleep()

    except:
        rospy.loginfo("Orca node terminated.")
