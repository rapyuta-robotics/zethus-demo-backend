#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
import message_filters


def cb_1(msg):
    global first_stamp, now
    for tf in msg.transforms:
        tf.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
    pub_1.publish(msg)


rospy.init_node('tf_republisher')
first_stamp = None

rospy.sleep(1)
now = rospy.Time.now()

pub_1 = rospy.Publisher('/tf', TFMessage, queue_size=1)

sub_1 = rospy.Subscriber('/tf_old', TFMessage, cb_1)

rospy.spin()