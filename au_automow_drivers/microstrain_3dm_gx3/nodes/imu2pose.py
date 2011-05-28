#!/usr/bin/env python
import roslib; roslib.load_manifest('microstrain_3dm_gx3')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

import numpy

import tf
import tf.transformations

pose_pub = None

def callback(data):
    global pose_pub
    msg = PoseStamped()
    msg.header = data.header
    msg.header.frame_id = "base_link"
    msg.pose.orientation = data.orientation
    (r,p,yaw) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, \
            data.orientation.z, data.orientation.w])
    yaw /= numpy.pi
    yaw *= 180.0
    rospy.loginfo("Heading in degrees: %f" % yaw)
    pose_pub.publish(msg)

def listener():
    global pose_pub
    rospy.init_node('imu2pose', anonymous=True)
    pose_pub = rospy.Publisher('/imu/pose', PoseStamped)
    rospy.Subscriber('/imu/data', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
