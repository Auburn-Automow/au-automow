#!/usr/bin/env python
import roslib; roslib.load_manifest('microstrain_3dm_gx3')
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

pose_pub = None

def callback(data):
    global pose_pub
    msg = PoseStamped()
    msg.header = data.header
    msg.pose.orientation = data.orientation
    pose_pub.publish(msg)

def listener():
    global pose_pub
    rospy.init_node('imu2pose', anonymous=True)
    pose_pub = rospy.Publisher('/imu/pose', PoseStamped)
    rospy.Subscriber('/imu/data', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
