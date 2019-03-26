#!/usr/bin/env python

import rospy
import sys
import random
import math

from sensor_msgs.msg import JointState, LaserScan

def main():
    for type in types:
        if type[0] == 'joint_state':
            publish_joint_state(types.index(type))
        elif type[0] == 'laser_scan':
            publish_laser_scan(types.index(type))

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
    laser_scan.angle_min = math.pi/3
    laser_scan.angle_max = (math.pi/3)*5
    laser_scan.angle_increment = math.pi/180
    laser_scan.scan_time = 0.1
    laser_scan.range_min = 0.1
    laser_scan.range_max = 2
    for i in range(int((laser_scan.angle_max-laser_scan.angle_min)/laser_scan.angle_increment)):
        laser_scan.ranges.append(abs(math.cos(i)))
        laser_scan.intensities.append(5)

    sensor_publisher[index].publish(laser_scan)


if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_sensor_publisher")

        types = [('joint_state',JointState), ('laser_scan',LaserScan)]
        sensor_publisher = []

        for type in types:
            sensor_publisher.append(rospy.Publisher(type[0], type[1], queue_size=1, latch=True))

        main()
    except rospy.ROSInterruptException:
        pass