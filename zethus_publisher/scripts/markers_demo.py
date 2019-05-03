#!/usr/bin/env python
import rospy
import sys
import random
from itertools import cycle

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

marker_id = 0

def main():
	while not rospy.is_shutdown():
		global marker_id
		marker_id = 0
		for type in types:
			marker = Marker()
			index = types.index(type)
			marker = get_marker_attributes(float(index))
			marker.pose.position.y = float(index) / 5
			make_and_publish_marker_aray(index, marker)
		demo_publisher.publish(demo_marker_array)
		del(demo_marker_array.markers[:])
		rospy.Rate(50).sleep()
		# rospy.spin()

def make_and_publish_marker_aray(index, marker):
	global marker_id
	display_label(marker, index)
	marker_array = MarkerArray()
	pose_x = 0.2
	count = 5
	for i in range(5):
		marker_element = Marker()
		marker_element = copy_marker_attributes(marker)
		marker_element.id = marker_id
		marker_element.pose.position.x = pose_x
		marker_element.scale.x /= i+1
		marker_element.scale.y /= i+1
		marker_element.scale.z /= i+1
		
		pct = float(i) / float(count)
		marker_element.color.r = pct * 1.0 + (1 - pct) * 0.0
		marker_element.color.g = pct * 0.0 + (1 - pct) * 1.0
		marker_element.color.b = pct * 0.0 + (1 - pct) * 0.0
		marker_element.color.a = 1.0

		demo_marker_array.markers.append(marker_element)
		marker_id += 1

		pose_x += 0.2

def display_label(marker=Marker(), index=0, text=None):
	global marker_id
	text_marker = Marker()
	text_marker = copy_marker_attributes(marker)
	# text_marker.header.frame_id = "world"
	text_marker.id = marker_id
	text_marker.type = marker.TEXT_VIEW_FACING
	text_marker.pose.position.y = float(index) / 5
	text_marker.scale.x = 0.04
	text_marker.scale.y = 0.04
	text_marker.scale.z = 0.04
	text_marker.color.a = 1.0
	text_marker.color.r = 1.0
	text_marker.color.g = 1.0
	text_marker.color.b = 1.0
	if text == None:
		text_marker.text = types[index][0]
	else:
		text_marker.text = text
	text_marker.pose.position.x = -0.25

	# display_text_publisher.publish(text_marker)
	demo_marker_array.markers.append(text_marker)
	marker_id += 1


def get_marker_attributes(index):
	marker = Marker()
	marker.header.frame_id = "world"
	marker.header.stamp = rospy.get_rostime()
	marker.id = index
	marker.type = index
	marker.action = marker.ADD  #Add/modify
	marker.scale.x = 0.05 
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 0.0
	marker.color.a = 1  # Don't forget to set the alpha!
	marker.color.r = 1
	# marker.color.g = 1
	# marker.color.b = random.uniform(0, 1)
	marker.text = "Demo"
	marker.mesh_resource = "package://zethus_publisher/data/meshes/forearm.dae"
	marker.mesh_use_embedded_materials = True
	del marker.points[:]
	del marker.colors[:]

	if index == marker.ARROW:
		marker.pose.orientation.y = -0.707
		marker.pose.orientation.w = 0.707
		marker.scale.x = 0.1
		marker.scale.y = 0.01
		marker.scale.z = 0.01

	elif index == marker.MESH_RESOURCE:
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2

	elif index in range(marker.CUBE_LIST, marker.POINTS + 1):
		marker.scale.x /= 5 if index != marker.POINTS else 10
		marker.scale.y /= 5 if index != marker.POINTS else 10
		marker.scale.z /= 5 if index != marker.POINTS else 10
		for x in range(0,5):
		  for y in range(0,5):
			for z in range(0,5):
			  p = Point()
			  p.x = x * 0.02
			  p.y = y * 0.02
			  p.z = z * 0.02	

			  marker.points.append(p)

			  c = ColorRGBA()
			  c.r = x * 0.2
			  c.g = y * 0.2
			  c.b = z * 0.2
			  c.a = 1.0
			  marker.colors.append(c)

	elif index == marker.TRIANGLE_LIST:
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		for x in range(0,2):  
		  for y in range(0,2):    
			for z in range(0,2): 
				p = Point()
				p.x = x * 0.4
				p.y = y * 0.4
				p.z = z * 0.4

				p2 = Point()
				p2.x = p.x + 0.2
				p2.y = p.y
				p2.z = p.z

				p3 = Point()
				p3.x = p2.x
				p3.y = p.y
				p3.z = p.z + 0.2

				marker.points.append(p)
				marker.points.append(p2)
				marker.points.append(p3)

				c = ColorRGBA()
				c.r = x * 0.5
				c.g = y * 0.5
				c.b = z * 0.5
				c.a = 1.0
				marker.colors.append(c)
				marker.colors.append(c)
				marker.colors.append(c)

	elif index == marker.LINE_LIST:
		count = 10
		marker.scale.x = 0.005
		for i in range(0,count):
			p1 = Point()
			p2 = Point()
			p1.x = 0
			p1.z = (i - count / 2) * 0.02
			p1.y = -0.05
			p2.x = 0
			p2.z = (i - count / 2) * 0.02
			p2.y = 0.05
			marker.points.append(p1)
			marker.points.append(p2)

			c = ColorRGBA()
			pct = float(i) / float(count)
			c.r = pct * 1.0 + (1 - pct) * 0.0
			c.g = pct * 0.0 + (1 - pct) * 0.0
			c.b = pct * 0.0 + (1 - pct) * 1.0
			c.a = 1.0
			marker.colors.append(c)
			marker.colors.append(c)

	elif index == marker.LINE_STRIP:
		marker.scale.x = 0.005
		for i in range (-5, 5):
			p = Point()
			p.y = (i % 2) * 0.05
			p.z = (i * 2) * 0.01
			p.x = 0;
			marker.points.append(p)

			c = ColorRGBA()
			pct = float(i + 5) / 10.0
			c.r = pct * 0.0 + (1 - pct) * 0.0
			c.g = pct * 1.0 + (1 - pct) * 0.0
			c.b = pct * 0.0 + (1 - pct) * 1.0
			c.a = 1.0
			marker.colors.append(c)


	return marker


def copy_marker_attributes(sample):
	marker = Marker()
	marker.header.frame_id = sample.header.frame_id
	marker.header.stamp = sample.header.stamp
	marker.id = sample.id
	marker.type = sample.type
	marker.action = sample.action
	marker.scale.x = sample.scale.x
	marker.scale.y = sample.scale.y
	marker.scale.z = sample.scale.z
	marker.pose.position.x = sample.pose.position.x
	marker.pose.position.y = sample.pose.position.y
	marker.pose.position.z = sample.pose.position.z
	marker.pose.orientation.x = sample.pose.orientation.x
	marker.pose.orientation.y = sample.pose.orientation.y
	marker.pose.orientation.z = sample.pose.orientation.z
	marker.pose.orientation.w = sample.pose.orientation.w
	marker.color.a = sample.color.a
	marker.color.r = sample.color.r
	marker.color.g = sample.color.g
	marker.color.b = sample.color.b
	marker.text = sample.text
	marker.mesh_resource = sample.mesh_resource
	marker.mesh_use_embedded_materials = sample.mesh_use_embedded_materials
	marker.points = sample.points
	marker.colors = sample.colors

	return marker

if __name__ == "__main__":
	try:
		rospy.init_node("zethus_backend_marker_publisher")

		cache = []
		types = [
			('arrow', Marker, MarkerArray), ('cube', Marker, MarkerArray),
			('sphere', Marker, MarkerArray), ('cylinder', Marker, MarkerArray),
			('line_strip', Marker, MarkerArray), ('line_list', Marker, MarkerArray),
			('cube_list', Marker, MarkerArray), ('sphere_list', Marker, MarkerArray),
			('points', Marker, MarkerArray), ('text_view_facing', Marker, MarkerArray),
			('mesh_resource', Marker, MarkerArray), ('triangle_list', Marker, MarkerArray),
		]
		marker_publisher = []
		marker_array_publisher = []

		demo_marker_array = MarkerArray()
		demo_publisher = rospy.Publisher('demo', MarkerArray, queue_size = 10, latch = True)
		main()
	except rospy.ROSInterruptException:
		pass