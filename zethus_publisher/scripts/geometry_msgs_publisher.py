#!/usr/bin/env python
import rospy
import random
import math
from itertools import cycle

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PolygonStamped, Polygon, Point32, WrenchStamped, AccelStamped, TwistStamped, Vector3Stamped, PointStamped
from visualization_msgs.msg import Marker


def main():
    iter = 0.0
    while not rospy.is_shutdown():
        #TODO: Change this logic to pass publisher index to the methods. This is ugly
        for type in types:
            index = types.index(type)
            if type[0] == 'pose_array':
                publish_pose_array(index, iter)
            elif type[0] == 'pose_stamped':
                publish_pose_stamped(index, iter)
            elif type[0] == 'polygon_stamped':
                publish_polygon_stamped(index, iter)
            elif type[0] == 'wrench_stamped':
                publish_wrench_stamped(index, iter)
            elif type[0] == 'accel_stamped':
                publish_accel_stamped(index, iter)
            elif type[0] == 'twist_stamped':
                publish_twist_stamped(index, iter)
            elif type[0] == 'vector3_stamped':
                publish_vector3_stamped(index, iter)
            elif type[0] == 'point_stamped':
                publish_point_stamped(index, iter)
        iter += 0.01
        rospy.Rate(100).sleep()


def publish_pose_array(index, iter):
    pose_array = PoseArray()
    pose_array.header.frame_id = 'world'
    pose_array.header.stamp = rospy.get_rostime()
    for i in range(10):
        if iter == 0.0:
            cache.append(random.randint(0, 5))
        pose = Pose()
        pose.position.x = math.sin(iter)  #iter
        pose.position.y = 2  #+ math.cos(iter) #(-1 if (iter % 2 == 0) else 1)
        pose.position.z = float(i) / 2
        pose.orientation.x = math.sin(iter)  #random.uniform(-2, 2)
        pose.orientation.y = cache[i] + math.cos(iter)  #(-2, 2)
        pose.orientation.z = cache[i] + 2 + math.cos(iter)  #(-2, 2)
        pose.orientation.w = 1
        pose_array.poses.append(pose)

    geometry_publisher[index].publish(pose_array)


def publish_pose_stamped(index, iter):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'world'
    pose_stamped.header.stamp = rospy.get_rostime()
    pose_stamped.pose.position.x = math.sin(iter)
    pose_stamped.pose.position.y = 5 + math.cos(iter)
    pose_stamped.pose.position.z = abs(math.cos(iter))
    pose_stamped.pose.orientation.x = math.sin(iter)
    pose_stamped.pose.orientation.y = math.cos(iter)
    pose_stamped.pose.orientation.z = math.cos(iter)
    pose_stamped.pose.orientation.w = 1

    geometry_publisher[index].publish(pose_stamped)


t = 0


def publish_polygon_stamped(index, iter):
    poly_stamped = PolygonStamped()
    poly_stamped.header.frame_id = 'polygon_frame'
    poly_stamped.header.stamp = rospy.get_rostime()
    global t
    dr = 0.1 * math.cos(t)
    radii = [0.25 - dr, 0.25 + dr]
    radius_index = 0
    num_points = 10
    for i in range(0, num_points):
        point = Point32()
        radius = radii[radius_index]
        radius_index = (radius_index + 1) % 2
        point.x = radius * math.cos(i * 2 * math.pi / num_points)
        point.y = radius * math.sin(i * 2 * math.pi / num_points)
        point.z = 0
        poly_stamped.polygon.points.append(point)

    t += .1

    geometry_publisher[index].publish(poly_stamped)


def publish_wrench_stamped(index, iter):
    wrench_stamped = WrenchStamped()
    wrench_stamped.header.frame_id = 'world'
    wrench_stamped.header.stamp = rospy.get_rostime()
    wrench_stamped.wrench.force.x = math.sin(iter) / 4
    wrench_stamped.wrench.force.y = math.cos(iter) / 4
    wrench_stamped.wrench.force.z = math.sin(iter) / 4
    wrench_stamped.wrench.torque.x = math.cos(iter) / 4
    wrench_stamped.wrench.torque.y = math.sin(iter) / 4
    wrench_stamped.wrench.torque.z = math.cos(iter) / 4

    geometry_publisher[index].publish(wrench_stamped)


def publish_accel_stamped(index, iter):
    accel_stamped = AccelStamped()
    accel_stamped.header.frame_id = 'world'
    accel_stamped.header.stamp = rospy.get_rostime()
    accel_stamped.accel.linear.x = random.uniform(-2, 2)
    accel_stamped.accel.linear.y = random.uniform(-2, 2)
    accel_stamped.accel.linear.z = random.uniform(-2, 2)
    accel_stamped.accel.angular.x = random.uniform(-2, 2)
    accel_stamped.accel.angular.y = random.uniform(-2, 2)
    accel_stamped.accel.angular.z = random.uniform(-2, 2)

    geometry_publisher[index].publish(accel_stamped)


def publish_twist_stamped(index, iter):
    twist_stamped = TwistStamped()
    twist_stamped.header.seq = 1
    twist_stamped.header.frame_id = 'world'
    twist_stamped.header.stamp = rospy.get_rostime()
    twist_stamped.twist.linear.x = random.uniform(-2, 2)
    twist_stamped.twist.linear.y = random.uniform(-2, 2)
    twist_stamped.twist.linear.z = random.uniform(-2, 2)
    twist_stamped.twist.angular.x = random.uniform(-2, 2)
    twist_stamped.twist.angular.y = random.uniform(-2, 2)
    twist_stamped.twist.angular.z = random.uniform(-2, 2)

    geometry_publisher[index].publish(twist_stamped)


def publish_vector3_stamped(index, iter):
    vector3_stamped = Vector3Stamped()
    vector3_stamped.header.seq = 1
    vector3_stamped.header.frame_id = 'world'
    vector3_stamped.header.stamp = rospy.get_rostime()
    vector3_stamped.vector.x = random.uniform(-2, 2)
    vector3_stamped.vector.y = random.uniform(-2, 2)
    vector3_stamped.vector.z = random.uniform(-2, 2)

    geometry_publisher[index].publish(vector3_stamped)


def publish_point_stamped(index, iter):
    point_stamped = PointStamped()
    point_stamped.header.frame_id = 'world'
    point_stamped.header.stamp = rospy.get_rostime()
    point_stamped.point.x = 0.0
    point_stamped.point.y = -0.25
    point_stamped.point.z = 0

    geometry_publisher[index].publish(point_stamped)


if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_geometry_publisher")

        cache = []
        types = [
            #('pose_array', PoseArray),
            ('pose_stamped', PoseStamped),
            ('polygon_stamped', PolygonStamped),
            ('wrench_stamped', WrenchStamped),
            ('accel_stamped', AccelStamped),
            ('twist_stamped', TwistStamped),
            ('vector3_stamped', Vector3Stamped),
            ('point_stamped', PointStamped)
        ]
        geometry_publisher = []

        for type in types:
            geometry_publisher.append(rospy.Publisher(type[0], type[1], queue_size=1, latch=True))

        main()
    except rospy.ROSInterruptException:
        pass
