#!/usr/bin/env python

import rospy
import sys
import random
import math

from sensor_msgs.msg import JointState, LaserScan, Range, MagneticField

def main():
    for type in types:
        index = types.index(type)
        if type[0] == 'joint_state':
            publish_joint_state(index)
        elif type[0] == 'laser_scan':
            publish_laser_scan(index)
        elif type[0] == 'range':
            publish_range(index)
        elif type[0] == 'magnetic_field':
            publish_magnetic_field(index)

    rospy.spin()

def publish_joint_state(index):
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    joint_state = JointState()
    joint_state.header.stamp = rospy.get_rostime()
    joint_state.name = joint_names
    for joint in joint_names:
        joint_state.position.append(random.uniform(-1.5707, 1.5707))
        joint_state.velocity.append(random.uniform(0.5, 1.5))
        joint_state.effort.append(random.uniform(-50, 50))

    sensor_publisher[index].publish(joint_state)

def publish_laser_scan(index):
    laser_scan = LaserScan()
    laser_scan.header.frame_id = 'world'
    laser_scan.header.stamp = rospy.get_rostime()
    laser_scan.angle_min = math.pi/6
    laser_scan.angle_max = (math.pi/6)*11
    laser_scan.angle_increment = math.pi/180
    laser_scan.scan_time = 0.1
    laser_scan.range_min = 0.1
    laser_scan.range_max = 2
    for i in range(int((laser_scan.angle_max-laser_scan.angle_min)/laser_scan.angle_increment)):
        laser_scan.ranges.append(1.5+abs(math.cos(i)))
        laser_scan.intensities.append(5)

    sensor_publisher[index].publish(laser_scan)

def publish_range(index):
    range = Range()
    range.header.frame_id = 'world'
    range.header.stamp = rospy.get_rostime()
    range.radiation_type = 0
    range.field_of_view = (math.pi/3)
    range.min_range = 0.1
    range.max_range = 2
    range.range = 1.5

    sensor_publisher[index].publish(range)

def publish_magnetic_field(index):
    field = MagneticField()
    field.header.frame_id = 'world'
    field.header.stamp = rospy.get_rostime()
    field.magnetic_field.x = random.uniform(-5, 5)
    field.magnetic_field.y = random.uniform(-5, 5)
    field.magnetic_field.z = random.uniform(-5, 5)
    for i in range(len(field.magnetic_field_covariance)):
        field.magnetic_field_covariance[i] = random.uniform(-5, 5)

    sensor_publisher[index].publish(field)

if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_sensor_publisher")

        types = [('joint_state',JointState), ('laser_scan',LaserScan), ('range', Range), ('magnetic_field', MagneticField)]
        sensor_publisher = []

        for type in types:
            sensor_publisher.append(rospy.Publisher(type[0], type[1], queue_size=1, latch=True))

        main()
    except rospy.ROSInterruptException:
        pass