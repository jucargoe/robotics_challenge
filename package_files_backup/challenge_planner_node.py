#!/usr/bin/env python

""" A ROS planning node that subscribes to a costmap
  and generates a path by using the Djikstra algorithm"""

import sys
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from challenge_dijkstra import Dijkstra


class Planner:
    def __init__(self):
        # As a first possibility, we will search for the init and the goal
        # at the parameter server
        self.goalx = None
        self.goaly = None
        self.initx = None
        self.inity = None
        self.map = None
        self.path = None

        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        rospy.Subscriber("/RvizListener/coordinates", PointStamped, self.coordinates)
        rospy.Subscriber('costmap_2d/costmap/costmap', OccupancyGrid, self.map_callback)
        self.listener = tf.TransformListener()
        self.init = False  # This flag would be raised in the map callback

    def coordinates(self, coordinates):
        if self.init is False:
            self.goalx = coordinates.point.x
            self.goaly = coordinates.point.y

    def odometry_callback(self, data):
        if self.init is False:
            self.initx = data.pose.pose.position.x
            self.inity = data.pose.pose.position.y

    def map_callback(self, map):
        self.map = map
        if self.init is False and self.goalx and self.initx:
            self.init = True
            self.path = self.calculate_path(self.initx, self.inity, self.goalx, self.goaly)
            self.publish_path_marker(self.path)

    def calculate_path(self, ix, iy, gx, gy):
        self.dijkstra = Dijkstra(self.map)
        return self.dijkstra.planning(ix, iy, gx, gy)

    def publish_path_marker(self, path):
        path_marker = Marker()
        x, y = path
        for i in range(len(x)):
            p = Point(x=x[i], y=y[i])
            path_marker.points.append(p)

        path_marker.points.reverse()
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.header.stamp = rospy.Time.now()
        path_marker.header.frame_id = "map"
        path_marker.color.a = 1.0
        path_marker.color.g = 1.0
        path_marker.scale.x = 0.2
        path_marker.lifetime.secs = 1000
        self.marker_pub.publish(path_marker)


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('Planning', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("Starting planning node. Waiting for valid map")

        planner = Planner()

        # The planning node will wait for new goals and the map at this rate
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            r.sleep()

    except:
        rospy.loginfo("Planning node terminated.")
