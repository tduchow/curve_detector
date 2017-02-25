#!/usr/bin/env python#!/usr/bin/env python

'''Listens to a LaserScan and finds curves, using algorithm from paper'''

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import MarkerArray, Marker

import math
import numpy


def callback(data):
	data.

def filter_intervals(ranges, intervals):
	min_length = 4
	filtered = filter(lambda t: abs(t[0]-t[1]) > min_length, intervals)
	#TODO: filter samples that are close to straight
	return filtered

#distance between two points in polar coordinates (angle in degrees, distance)
def polar_distance(p1, p2):
	return math.sqrt(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*math.cos(math.radians(p1[0]-p2[0])))

#return the angle at p2 between p1 and p3, in degrees
def internal_angle(p1, p2, p3):
	a = polar_distance(p1, p2)
	c = polar_distance(p1, p3)
	b = polar_distance(p2, p3)
	return math.degrees(math.acos((a**2+b**2-c**2)/(2*a*b)))
	
def make_marker(arc):
	m = Marker()
	m.header.frame_id = "base_link";
	m.header.stamp = rospy.get_rostime();
	m.type = Marker.CYLINDER;
	m.action = Marker.ADD;
	m.pose.position.x = arc[0][1]*math.cos(math.radians(arc[0][0]));
	m.pose.position.y = arc[0][1]*math.sin(math.radians(arc[0][0]));

	m.scale.x = arc[1];
	m.scale.y = arc[1];
	m.scale.z = 1;
	m.color.a = 1.0; 

	m.color.g = 1.0;
	return m

	

def curve_detector():

	global pub 
	pub = rospy.Publisher('arc_markers', MarkerArray, queue_size=10)
    
	rospy.init_node('arc_marker')

	rospy.Subscriber("/arcs", PointArray, callback)


    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':

	curve_detector()
