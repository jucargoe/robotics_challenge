#!/usr/bin/env python

import sys
import math
import rospy
import tf
import numpy as np
import challenge_pyorca as pyorca
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from challenge_halfplaneintersect import InfeasibleError

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
        self.agents = []
        self.vel_max = 0.5
        self.angular_max = 0.3
        self.angular_min = -0.3
        self.contador = 0

        rospy.Subscriber("/ScanDownsampler/laser", LaserScan, self.scan)
        rospy.Subscriber("/ScanDownsampler/marker", Marker, self.marker)
        rospy.Subscriber("/RvizListener/coordinates", PointStamped, self.coordinates)
        rospy.Subscriber('/odom', Odometry, self.callback)

    def callback(self, odometry):
        # print(odometry)
        self.robot_position_x = odometry.pose.pose.position.x
        self.robot_position_y = odometry.pose.pose.position.y
        # print(str(self.robot_position_x), str(self.robot_position_y))

    def marker(self, marker_data):
        self.agents = []
        for point in marker_data.points:
            position = (point.x, point.y)
            velocity = (0., 0.)
            radius = .01
            max_speed = 0
            pref_velocity = np.array([0., 0.])
            agent = pyorca.Agent(position, velocity, radius, max_speed, pref_velocity)
            self.agents.append(agent)

    def scan(self, laser_data):
        if self.linear > 0.0 and self.contador == 0:
            position = (0., 0.)
            velocity = (self.linear*np.cos(self.angular), self.linear*np.sin(self.angular))
            radius = 0.3
            max_speed = 0.5
            pref_velocity = np.array(velocity)
            robot = pyorca.Agent(position, velocity, radius, max_speed, pref_velocity)
            
            try:
                vel, lines = pyorca.orca(robot, self.agents, 2, 0.5)
                velocidad = [np.sqrt(vel[0]**2 + vel[1]**2), np.arctan2(vel[1], vel[0])]

                self.linear = velocidad[0]
                if self.linear > self.vel_max:
                    self.linear = self.vel_max

                self.angular = velocidad[1]
                if self.angular > self.angular_max:
                    self.angular = self.angular_max
                elif self.angular < self.angular_min:
                    self.angular = self.angular_min
                    
            except InfeasibleError:
                self.contador = 5
                self.linear = -0.2
                self.angular = 0

            # print("vel: " + str(self.linear))
            # print("ang: " + str(self.angular))
        elif self.contador > 0:
            self.contador -= 1
            self.linear = -0.2
            self.angular = 0
            
        self.publish(self.linear, self.angular)

    def coordinates(self, coordinates):
        self.goal_x = coordinates.point.x
        self.goal_y = coordinates.point.y

    def command(self):
        # rospy.loginfo("Command")

        if self.goal_x != np.Infinity or self.goal_y != np.Infinity:
            #print("x = " + str(self.goal_x) + ", y = " + str(self.goal_y))
            distance = math.sqrt((self.goal_x) ** 2 + (self.goal_y) ** 2)

            if distance > 0.1:
                self.angular = math.atan2(self.goal_y, self.goal_x) / 2.0
                if self.angular > 0.3:
                    self.angular = 0.3
                elif self.angular < -0.3:
                    self.angular = -0.3

                if self.angular < 0.3 and self.angular > -0.3:
                    self.linear = 0.1 if self.linear == 0.0 else self.linear + 0.05

                    if self.linear > 0.5:
                        self.linear = 0.5
                else:
                    self.linear -= 0.05
                    if self.linear < 0.0:
                        self.linear = 0.0
            else:
                self.angular = 0.0
                self.linear = 0.0

            # print("distance: " + str(distance) + ", angular: " + str(self.angular) + ", linear: " + str(self.linear))
            # self.publish(self.linear, self.angular)

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
