#!/usr/bin/env python
import rospy
import random
from nav_msgs.msg import Odometry, OccupancyGrid


def main():
    #TODO: Change this logic to pass publisher index to the methods. This is ugly
    for type in types:
        index = types.index(type)
        if type[0] == 'odometry':
            publish_odometry(index)
        elif type[0] == 'occupancy_grid':
            publish_occupancy_grid(index)

    rospy.spin()


def publish_odometry(index):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "world"
    odom.pose.pose.position.x = random.uniform(-2, 2)
    odom.pose.pose.position.y = random.uniform(-2, 2)
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation.x = random.uniform(-2, 2)
    odom.pose.pose.orientation.y = random.uniform(-2, 2)
    odom.pose.pose.orientation.z = random.uniform(-2, 2)
    odom.pose.pose.orientation.w = random.uniform(1, 2)

    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = random.uniform(-2, 2)
    odom.twist.twist.linear.y = random.uniform(-2, 2)
    odom.twist.twist.linear.z = random.uniform(-2, 2)
    odom.twist.twist.angular.x = random.uniform(-2, 2)
    odom.twist.twist.angular.y = random.uniform(-2, 2)
    odom.twist.twist.angular.z = random.uniform(-2, 2)

    nav_publisher[index].publish(odom)


def publish_occupancy_grid(index):
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "world"
    occupancy_grid.info.origin.position.x = 0
    occupancy_grid.info.origin.position.y = 0
    occupancy_grid.info.origin.position.z = 0
    occupancy_grid.info.origin.orientation.x = 0
    occupancy_grid.info.origin.orientation.y = 0
    occupancy_grid.info.origin.orientation.z = 0
    occupancy_grid.info.origin.orientation.w = 1
    occupancy_grid.info.width = 5
    occupancy_grid.info.height = 5
    occupancy_grid.info.resolution = 1
    for i in range(occupancy_grid.info.width * occupancy_grid.info.height):
        occupancy_grid.data.append(random.randint(0, 100))

    nav_publisher[index].publish(occupancy_grid)


if __name__ == "__main__":
    try:
        rospy.init_node("zethus_backend_nav_publisher")

        types = [
            #('odometry', Odometry), 
            ('occupancy_grid', OccupancyGrid)
        ]
        nav_publisher = []

        for type in types:
            nav_publisher.append(rospy.Publisher(type[0], type[1], queue_size=1, latch=True))

        main()
    except rospy.ROSInterruptException:
        pass