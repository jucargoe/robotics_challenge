#!/usr/bin/env python

# A basic tool that downsamples a radar scan by taking one point for each n

import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class ScanDownsampler:
    def __init__(self):
        self.n = rospy.get_param("~n", default=8)
        self.scale = rospy.get_param("~scale", default=0.3)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        self.marker_pub = rospy.Publisher("~marker", Marker, queue_size=10, latch=True)
        self.laser_pub = rospy.Publisher("~laser", LaserScan, queue_size=10, latch=True)

    def callback(self, data):
        marker = Marker()
        marker.header = data.header #_id = data.header.frame_id
        marker.header.stamp = rospy.Time.now()
        angle = 0

        marker.id = 0
        marker.ns = "scan_ds"
        marker.lifetime.secs = 100

        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale

        data.ranges = list(data.ranges)
        for i in range(0,len(data.ranges)):
            if i % self.n != 0 or np.cos(angle) < 0:
                data.ranges[i] = np.Infinity

            angle += data.angle_increment

            
        angle = 0
        for i in range(0,len(data.ranges), self.n):
            if np.abs(data.ranges[i]) < 20.0:
                
                p = Point()
                p.x = data.ranges[i] * np.cos(angle)
                p.y = data.ranges[i] * np.sin(angle)
                p.z = 0
                marker.points.append(p)


            angle += self.n * data.angle_increment
            
        marker.type = marker.POINTS
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.b = 0.0
        marker.color.g = 0.0

        self.marker_pub.publish(marker)
        self.laser_pub.publish(data)
    
 
if __name__ == '__main__':
    try:
	    # initiliaze
        rospy.init_node('ScanDownsampler', anonymous=False)

        # Instantiate downsampler
        ds=ScanDownsampler()

	    # Log initialization and spin
        rospy.loginfo("Scan Downsampler starting. N = %d", ds.n)
        rospy.spin()
    except:
        rospy.loginfo("ScanDownsampler node terminated.")
