#!/usr/bin/env python

import sys
import math
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan

class Orca():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.goal_x = np.Infinity
        self.goal_y = np.Infinity
        rospy.Subscriber("/ScanDownsampler/laser", LaserScan, self.scan)
        rospy.Subscriber("/RvizListener/coordinates", PointStamped, self.coordinates)

    def scan(self, laser_data):
        print("scan")
        #print(laser_data)

    def coordinates(self, coordinates):
        self.goal_x = coordinates.point.x
        self.goal_y = coordinates.point.y

    def command(self):
        #rospy.loginfo("Command")
        if self.goal_x != np.Infinity or self.goal_y != np.Infinity:
            #print("x = " + str(self.goal_x) + ", y = " + str(self.goal_y))
            angular = 0.0
            linear = 0.0
            distance = math.sqrt((self.goal_x) ** 2 + (self.goal_y) ** 2)

            if distance > 0.1:
                angular = math.atan2(self.goal_y, self.goal_x) / 2.0

                if angular < 0.1 and angular > -0.1:
                    linear = (distance * 0.5) / 1.0
                    
                """if angular > 0.3:
                    angular = 0.3
                if angular < -0.3:
                    angular = -0.3
                if linear > 0.5:
                    linear = 0.5"""

            print("distance: " + str(distance) + ", angular: " + str(angular) + ", linear: " + str(linear))
            self.publish(linear, angular)

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
