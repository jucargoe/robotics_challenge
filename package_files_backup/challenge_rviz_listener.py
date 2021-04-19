#!/usr/bin/env python

import sys
import math
import rospy
import tf
import collections
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Twist

class RvizListener():

    def __init__(self):
        self.coordinates_publisher = rospy.Publisher('~coordinates', PointStamped, queue_size=10)
        self.listener = tf.TransformListener()
        self.goal_x = np.Infinity
        self.goal_y = np.Infinity
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.coordinates)
        
    def coordinates(self, data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y
        rospy.loginfo("coordenadas rviz recibidas: x = %f y = %f", self.goal_x, self.goal_y)

    def command(self):
        #rospy.loginfo("Command")
        if self.goal_x != np.Infinity or self.goal_y != np.Infinity:
            goal = PointStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time()
            goal.point.x = self.goal_x
            goal.point.y = self.goal_y
            goal.point.z = 0.0
            # print("x = " + str(self.goal_x) + ", y = " + str(self.goal_y))

            base_goal = PointStamped()
            try:
                base_goal = self.listener.transformPoint('base_footprint', goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return

            self.publish(base_goal)

    def publish(self, base_goal):
        print("publicando base_goal")
        self.coordinates_publisher.publish(base_goal)

    def shutdown(self):
        rospy.loginfo("Stop RvizListener")
        self.coordinates_publisher.publish(PointStamped())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('RvizListener', anonymous=False)
        rospy.loginfo("To stop RvizListener CTRL + C")
        rviz = RvizListener()
        rospy.on_shutdown(rviz.shutdown)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            rviz.command()
            r.sleep()
    except:
        rospy.loginfo("RvizListener node terminated.")
