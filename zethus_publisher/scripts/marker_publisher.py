#!/usr/bin/env python

import rospy
import sys
import random

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def main():
    publish_marker()
    publish_marker_array()

    rospy.spin()

def publish_marker():
    for type in types:
        index = types.index(type)
        marker = Marker()
        marker.header.frame_id = "world"
        marker.id = 0
        marker.type = index 
        marker.action = marker.ADD #Add/modify
        marker.pose.position.x = 0.0
        marker.pose.position.y = index*2
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.text = "Demo"
        marker.mesh_resource = "package://ur_description/meshes/ur10/visual/forearm.dae";
        marker.mesh_use_embedded_materials = True

        for i in range(6):
            point = Point()
            point.x += i
            point.y = random.uniform(-0.4, 0.4)
            point.z = 0
            marker.points.append(point)

            # color = ColorRGBA()
            # color.r = 1.0
            # color.g = 0.0
            # color.b = 0.0
            # color.a = 1.0
            # marker.colors.append(color)

        marker_publisher[index].publish(marker)

def publish_marker_array():
    for type in types:
        index = types.index(type)
        marker_array = MarkerArray()
        pose_z = 1.0
        for i in range(10):
            marker_element = Marker()
            marker_element.header.frame_id = "world"
            marker_element.id = i
            marker_element.type = index #CUBE
            marker_element.action = marker_element.ADD #Add/modify
            marker_element.pose.position.x = 0
            marker_element.pose.position.y = index*2
            marker_element.pose.position.z = pose_z
            marker_element.pose.orientation.x = 0.0
            marker_element.pose.orientation.y = 0.0
            marker_element.pose.orientation.z = 0.0
            marker_element.pose.orientation.w = 1.0
            marker_element.scale.x = 0.1
            marker_element.scale.y = 0.1
            marker_element.scale.z = 0.1
            marker_element.color.a = 1.0 # Don't forget to set the alpha!
            marker_element.color.r = 0.0
            marker_element.color.g = 1.0
            marker_element.color.b = 0.0
            marker_element.text = "Demo"
            marker_element.mesh_resource = "package://ur_description/meshes/ur10/visual/forearm.dae";
            marker_element.mesh_use_embedded_materials = True

            for i in range(6):
                point = Point()
                point.x += i
                point.y = random.uniform(-0.4, 0.4)
                point.z = 0
                marker_element.points.append(point)

                # color = ColorRGBA()
                # color.r = 1.0
                # color.g = 0.0
                # color.b = 0.0
                # color.a = 1.0
                # marker.colors.append(color)

            marker_array.markers.append(marker_element)

            pose_z += 0.5

        marker_array_publisher[index].publish(marker_array)

if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_sample_publisher")

        types = ['arrow', 'cube', 'sphere', 'cylinder', 'line_strip', 'line_list', 'cube_list', 'sphere_list', 'points', 'text_view_facing', 'mesh_resource', 'triangle_list']
        marker_publisher = []
        marker_array_publisher = []

        for type in types:
            marker_publisher.append(rospy.Publisher(type, Marker, queue_size=10, latch=True))
            marker_array_publisher.append(rospy.Publisher(type+'_array', MarkerArray, queue_size=10, latch=True))

        main()
    except rospy.ROSInterruptException:
        pass