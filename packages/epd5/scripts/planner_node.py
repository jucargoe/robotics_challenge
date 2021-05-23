#!/usr/bin/env python

""" A ROS planning node that subscribes to a costmap
  and generates a path by using the Djikstra algorithm"""

import sys
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from dijkstra import Dijkstra


class Planner:
    def __init__(self):
        # As a first possibility, we will search for the init and the goal
        # at the parameter server
        rospy.loginfo("In the planner constructor")
        if rospy.has_param('~init'):
            self.initx = rospy.get_param('~init/x')
            self.inity = rospy.get_param('~init/y')

        if rospy.has_param('~goal'):
            self.goalx = rospy.get_param('~goal/x')
            self.goaly = rospy.get_param('~goal/y')
        rospy.loginfo("Here")
        print('Init (%f, %f). Goal (%f,%f): '%(self.initx, self.inity, self.goalx, self.goaly))

        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=10)
        rospy.Subscriber('costmap_2d/costmap/costmap', OccupancyGrid, self.map_callback)
        rospy.Subscriber('goal_pose', PoseStamped, self.goal_callback)
        self.listener = tf.TransformListener()
        self.init = False  # This flag would be raised in the map callback

    def map_callback(self, map):
        self.map = map
        print(map)
        if self.init == False:
            self.init = True
            self.path = self.calculate_path(self.initx, self.inity, self.goalx, self.goaly)
            self.publish_path_marker(self.path)
            print(self.path)
            # TODO: 2 add the path publisher to nav_msgs/Path. Hint: use the pyyaml module
            # More information: https://stackabuse.com/reading-and-writing-yaml-to-a-file-in-python/

    def goal_callback(self, map):
        # TODO
        print("goal_callback todo")

    def calculate_path(self, ix, iy, gx, gy):
        self.dijkstra = Dijkstra(self.map)
        return self.dijkstra.planning(ix, iy, gx, gy)

    def publish_path_marker(self, path):
        # TODO: 1 complete the publish marker method by publishing a LINE_STRIP marker
        # Beware of the header (timestamp and frame_id)
        # rospy.loginfo('Called publish path marker. TODO: complete it!')
        path_marker = Marker()
        x, y = path
        for i in range(len(x)):
            p = Point(x=x[i], y=y[i])
            path_marker.points.append(p)

        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.header.stamp = rospy.Time.now()
        path_marker.header.frame_id = "map"
        path_marker.color.a = 1.0
        path_marker.color.g = 1.0
        path_marker.scale.x = 0.2
        path_marker.lifetime.secs = 1000
        self.marker_pub.publish(path_marker)
        rospy.loginfo("Marker published!")


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('planning', anonymous=False)

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
