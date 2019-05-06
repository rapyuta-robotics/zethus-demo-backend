#!/usr/bin/env python
import rospy
import sys
import random
from itertools import cycle

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def main():
    for i in cycle(range(5)):
        if not rospy.is_shutdown():
            iter = float(i)
            publish_marker(iter)
            publish_marker_array(iter)
            rospy.Rate(1).sleep()
        else:
            break


def publish_marker(iter):
    for type in types:
        index = types.index(type)
        marker = get_marker_attributes(index, iter)

        marker_publisher[index].publish(marker)


def publish_marker_array(iter):
    for type in types:
        index = types.index(type)
        marker_array = MarkerArray()
        pose_z = 6.0
        marker_sample = get_marker_attributes(index, iter)
        for i in range(10):
            marker_element = copy_marker_attributes(marker_sample)
            marker_element.id = i
            marker_element.pose.position.z = pose_z

            marker_array.markers.append(marker_element)

            pose_z += 4.0

        marker_array_publisher[index].publish(marker_array)


def get_marker_attributes(index, iter):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.get_rostime()
    marker.id = index
    marker.type = index
    marker.action = marker.ADD  #Add/modify
    marker.scale.x = 0.2 + (iter / 5)
    marker.scale.y = 0.2 + (iter / 5)
    marker.scale.z = 0.2 + (iter / 5)
    marker.pose.position.x = iter
    marker.pose.position.y = index * 10 + (-1 if (iter % 2 == 0) else 1)
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = random.uniform(-2, 2)
    marker.pose.orientation.y = random.uniform(-2, 2)
    marker.pose.orientation.z = random.uniform(-2, 2)
    marker.pose.orientation.w = 1.0
    marker.color.a = random.uniform(0.5, 1)  # Don't forget to set the alpha!
    marker.color.r = random.uniform(0, 1)
    marker.color.g = random.uniform(0, 1)
    marker.color.b = random.uniform(0, 1)
    marker.text = "Demo"
    marker.mesh_resource = "package://zethus_publisher/data/meshes/forearm.dae"
    marker.mesh_use_embedded_materials = True

    for i in range(6):
        point = Point()
        point.x += i if marker.type != marker.TRIANGLE_LIST else 2 * i
        point.y = random.uniform(-2, 2)
        point.z = 0
        marker.points.append(point)

        color = ColorRGBA()
        color.r = random.uniform(0, 1)
        color.g = random.uniform(0, 1)
        color.b = random.uniform(0, 1)
        color.a = random.uniform(0.5, 1)
        marker.colors.append(color)

    # Special cases for better visualization
    # TODO: Move to a separate function
    if marker.type in (4, 5, 8):
        marker.scale.x = 0.1 + (iter / 20)
        marker.scale.y = 0.1 + (iter / 20)

    if marker.type == marker.LINE_STRIP:
        del cache[:]
        cache.append(marker.points)
        cache.append(marker.colors)
        cache.append(marker.pose.orientation)
    if marker.type == marker.LINE_LIST:
        marker.points = cache[0]
        marker.colors = cache[1]
        marker.pose.orientation = cache[2]

    if marker.type == marker.MESH_RESOURCE:
        marker.scale.x = 2 + (iter / 5)
        marker.scale.y = 2 + (iter / 5)
        marker.scale.z = 2 + (iter / 5)

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
            ('mesh_resource', Marker, MarkerArray), ('triangle_list', Marker, MarkerArray)
        ]
        marker_publisher = []
        marker_array_publisher = []

        for type in types:
            marker_publisher.append(rospy.Publisher(type[0], type[1], queue_size=1, latch=True))
            marker_array_publisher.append(
                rospy.Publisher('array_' + type[0], type[2], queue_size=1, latch=True)
            )

        main()
    except rospy.ROSInterruptException:
        pass