#!/usr/bin/env python
import roslib; roslib.load_manifest('microstrain_3dm_gx3')
import rospy
from sensor_msgs.msg import Imu

imu_pub = None
count = 1

def callback(data):
    global imu_pub, count
    if count == 1:
        imu_pub.publish(data)
        count = 10
    else:
        count -= 1

def listener():
    global imu_pub
    rospy.init_node('decimate_imu', anonymous=True)
    imu_pub = rospy.Publisher('/imu/data', Imu)
    rospy.Subscriber('/imu/data_fast', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
