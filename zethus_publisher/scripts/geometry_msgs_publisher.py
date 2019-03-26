#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PolygonStamped, Polygon, Point32, WrenchStamped, AccelStamped, TwistStamped, Vector3Stamped

def main():
    for type in types:
        index = types.index(type)
        if type[0] == 'pose_array':
            publish_pose_array(index)
        elif type[0] == 'pose_stamped':
            publish_pose_stamped(index)
        elif type[0] == 'polygon_stamped':
            publish_polygon_stamped(index)
        elif type[0] == 'wrench_stamped':
            publish_wrench_stamped(index)
        elif type[0] == 'accel_stamped':
            publish_accel_stamped(index)
        elif type[0] == 'twist_stamped':
            publish_twist_stamped(index)
        elif type[0] == 'vector3_stamped':
            publish_vector3_stamped(index)

    rospy.spin()

def publish_pose_array(index):
    pose_array = PoseArray()
    pose_array.header.frame_id = 'world'
    pose_array.header.stamp = rospy.get_rostime()
    for i in range(10):
        pose = Pose()
        pose.position.x = i 
        pose.position.y = 3
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0
        pose_array.poses.append(pose)
    
    geometry_publisher[index].publish(pose_array)

def publish_pose_stamped(index):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'world'
    pose_stamped.header.stamp = rospy.get_rostime()
    pose_stamped.pose.position.x = 3
    pose_stamped.pose.position.y = 4
    pose_stamped.pose.position.z = 0
    pose_stamped.pose.orientation.x = 0
    pose_stamped.pose.orientation.y = 0
    pose_stamped.pose.orientation.z = 0
    pose_stamped.pose.orientation.w = 0

    geometry_publisher[index].publish(pose_stamped)


def publish_polygon_stamped(index):
    poly_stamped = PolygonStamped()
    poly_stamped.header.frame_id = 'world'
    poly_stamped.header.stamp = rospy.get_rostime()
    poly = Polygon()
    for i in range(4):
        point =Point32()
        point.x = random.uniform(-2,2)
        point.y = random.uniform(-2,2) - 2.0
        point.z = random.uniform(-2,2)
        poly.points.append(point)
    poly_stamped.polygon = poly

    geometry_publisher[index].publish(poly_stamped)


def publish_wrench_stamped(index):
    wrench_stamped = WrenchStamped()
    wrench_stamped.header.frame_id = 'world'
    wrench_stamped.header.stamp = rospy.get_rostime()  
    wrench_stamped.wrench.force.x = random.uniform(-2,2)
    wrench_stamped.wrench.force.y = random.uniform(-2,2)
    wrench_stamped.wrench.force.z = random.uniform(-2,2)
    wrench_stamped.wrench.torque.x = random.uniform(-2,2)
    wrench_stamped.wrench.torque.y = random.uniform(-2,2)
    wrench_stamped.wrench.torque.z = random.uniform(-2,2)

    geometry_publisher[index].publish(wrench_stamped)

def publish_accel_stamped(index):
    accel_stamped = AccelStamped()
    accel_stamped.header.frame_id = 'world'
    accel_stamped.header.stamp = rospy.get_rostime()
    accel_stamped.accel.linear.x = random.uniform(-2,2)
    accel_stamped.accel.linear.y = random.uniform(-2,2)
    accel_stamped.accel.linear.z = random.uniform(-2,2)
    accel_stamped.accel.angular.x = random.uniform(-2,2)
    accel_stamped.accel.angular.y = random.uniform(-2,2)
    accel_stamped.accel.angular.z = random.uniform(-2,2)

    geometry_publisher[index].publish(accel_stamped)

def publish_twist_stamped(index):
    twist_stamped = TwistStamped()
    twist_stamped.header.seq = 1
    twist_stamped.header.frame_id = 'world'
    twist_stamped.header.stamp = rospy.get_rostime()
    twist_stamped.twist.linear.x = random.uniform(-2,2)
    twist_stamped.twist.linear.y = random.uniform(-2,2)
    twist_stamped.twist.linear.z = random.uniform(-2,2)
    twist_stamped.twist.angular.x = random.uniform(-2,2)
    twist_stamped.twist.angular.y = random.uniform(-2,2)
    twist_stamped.twist.angular.z = random.uniform(-2,2)

    geometry_publisher[index].publish(twist_stamped)

def publish_vector3_stamped(index):
    vector3_stamped = Vector3Stamped()
    vector3_stamped.header.seq = 1
    vector3_stamped.header.frame_id = 'world'
    vector3_stamped.header.stamp = rospy.get_rostime()
    vector3_stamped.vector.x = random.uniform(-2,2)
    vector3_stamped.vector.y = random.uniform(-2,2)
    vector3_stamped.vector.z = random.uniform(-2,2)

    geometry_publisher[index].publish(vector3_stamped)


if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_geometry_msg_publisher")

        types = [('pose_array',PoseArray), ('pose_stamped',PoseStamped), ('polygon_stamped', PolygonStamped), ('wrench_stamped', WrenchStamped) ,('accel_stamped',AccelStamped),('twist_stamped', TwistStamped),('vector3_stamped', Vector3Stamped)]
        geometry_publisher = []

        for type in types:
            geometry_publisher.append(rospy.Publisher(type[0], type[1], queue_size=1, latch=True))

        main()
    except rospy.ROSInterruptException:
        pass

