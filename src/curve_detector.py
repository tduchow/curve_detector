#!/usr/bin/env python

'''Listens to a LaserScan and finds curves, using algorithm from paper'''

import rospy
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

import math
import numpy


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Received a scan")
	range_pairs = list(enumerate(data.ranges))#now pairs of (degree angle, range)
	print range_pairs
	good_ranges = filter(lambda tup: tup[1] < data.range_max, range_pairs)#ranges with valid data

	#find intervals
	intervals = [[0, 0]]#index of first and last range of intervals
	
	#last_range = good_ranges[0][1]
	for i, (t, r) in enumerate(good_ranges):#i is merely index of the valid ranges, not connected to angle
		if i>0:
			print t, r
			delta = abs(good_ranges[i-1][1] - r)
			if delta > 0.1: 
				print 'interval'
				#ok, so this point is the beginning of a new interval
				print 'an interval just ended at {}'.format(good_ranges[i-1][0])
				print 'an interval just started at {}'.format(t)
				intervals[-1][1] = good_ranges[i-1][0] #look back a point, close the last interval
				intervals.append([t, 0]) #add the angle of the start of this the interval
			last_range = r

	intervals[-1][1] = good_ranges[-1][0]#TODO: stitch the beginning and the end of the scan together

	print intervals 
	#filter intervals
	good_intervals = filter_intervals(good_ranges, intervals)
	print good_intervals
	
	arcs = []
	pc = PointCloud()
	pc.header.frame_id = "laser";
	pc.header.stamp = rospy.get_rostime();
	pc.channels.append(ChannelFloat32())
	pc.channels[0].name = "radius"
	pc.channels.append(ChannelFloat32())
	pc.channels[1].name = "internal_angle"
	pc.channels.append(ChannelFloat32())
	pc.channels[2].name = "std_dev"
	for begin_th, end_th in good_intervals:
		#could also just filter as we go here
		first_pt = range_pairs[begin_th]#index back into original data
		last_pt = range_pairs[end_th]
		between_pts = range_pairs[begin_th+1:end_th]#this will contain invalid ranges, TODO: how to only look at good, filtered pts
		angles = []
		for pt in between_pts:
			if pt[1] < data.range_max:
				angles.append(internal_angle(first_pt, pt, last_pt))
		std_dev = numpy.std(angles)
		mean = numpy.mean(angles)
		if 70 < mean and mean < 135 and std_dev < 15:
			print 'possibly arc found at ', begin_th, end_th
			print first_pt, last_pt
			center_range = (first_pt[1] + last_pt[1])/2
			center_angle = (begin_th + end_th)/2
			radius = polar_distance(first_pt, last_pt)/2
			
			pc.points.append(makePoint32Polar((center_angle, center_range)))
			
			#pc.points.append(makePoint32Polar(first_pt))
			
			#pc.points.append(makePoint32Polar(last_pt))
			pc.channels[0].values.append(radius)
			pc.channels[1].values.append(mean)
			pc.channels[2].values.append(std_dev)
	
	pub.publish(pc)


def filter_intervals(ranges, intervals):
	min_length = 4 
	filtered = filter(lambda t: abs(t[0]-t[1]) > min_length, intervals)
	#TODO: filter samples that are close to straight
	return filtered

#distance between two points in polar coordinates (angle in degrees, distance)
def polar_distance(p1, p2):
	return math.sqrt(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*math.cos(math.radians(p1[0]-p2[0])))

def polar_to_rect_x(p):
	return -p[1]*math.cos(math.radians(p[0]))#negating x fixes coordinate transform TODO: use ROS transforms 

def polar_to_rect_y(p):
	return p[1]*math.sin(math.radians(p[0]))

#return the angle at p2 between p1 and p3, in degrees
def internal_angle(p1, p2, p3):
	a = polar_distance(p1, p2)
	c = polar_distance(p1, p3)
	b = polar_distance(p2, p3)
	return math.degrees(math.acos((a**2+b**2-c**2)/(2*a*b)))

def makePoint32Polar(p):
	return Point32(polar_to_rect_x(p), polar_to_rect_y(p), 0)
	

def curve_detector():

	global pub 
	pub = rospy.Publisher('arcs', PointCloud, queue_size=10)
	rospy.loginfo('curve node coming online')
    
	rospy.init_node('curve_detector')

	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.loginfo("Curve Detector subscribed")

    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':

	curve_detector()
