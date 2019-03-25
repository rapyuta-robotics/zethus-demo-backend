#!/usr/bin/env python
import rospy
import sys

from visualization_msgs.msg import Marker, MarkerArray

def main():
    publish_marker()
    publish_marker_array()

    rospy.spin()

def publish_marker():
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = 0
    marker.type = 1 #CUBE
    marker.action = marker.ADD #Add/modify
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
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

    marker_publisher.publish(marker)

def publish_marker_array():
    marker_array = MarkerArray()
    pose_x = 1.0
    for i in range(10):
        marker_element = Marker()
        marker_element.header.frame_id = "world"
        marker_element.id = i
        marker_element.type = 1 #CUBE
        marker_element.action = marker_element.ADD #Add/modify
        marker_element.pose.position.x = pose_x
        marker_element.pose.position.y = 0
        marker_element.pose.position.z = 0
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
        marker_array.markers.append(marker_element)

        pose_x += 0.5

    marker_array_publisher.publish(marker_array)

if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_sample_publisher")


        marker_publisher = rospy.Publisher('cube', Marker, queue_size=10, latch=True)
        marker_array_publisher = rospy.Publisher('cubes', MarkerArray, queue_size=10, latch=True)

        main()
    except rospy.ROSInterruptException:
        pass
