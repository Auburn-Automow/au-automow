#!/usr/bin/env python
import roslib; roslib.load_manifest('microstrain_3dm_gx3')
import rospy
from sensor_msgs.msg import Imu
import tf
import math

global imu_repub 
imu_repub = None 

def wrapToPi(angle):
    angle += math.pi
    is_neg = (angle < 0)
    angle = math.fmod(angle,(2.0*math.pi))
    if (is_neg):
        angle += 2.0 * math.pi
    angle -= math.pi
    return angle

def imuDataReceived(data):
    global imu_repub

    angles = tf.transformations.euler_from_quaternion(
            [data.orientation.x, 
            data.orientation.y, 
            data.orientation.z, 
            data.orientation.w])
    imu_msg = Imu()
    
    yaw = wrapToPi(angles[2] - math.pi/2)

    orientation = tf.transformations.quaternion_from_euler(0,0,yaw)
    imu_msg.orientation.x = orientation[0]
    imu_msg.orientation.y = orientation[1]
    imu_msg.orientation.z = orientation[2]
    imu_msg.orientation.w = orientation[3]
    imu_msg.header = data.header
    imu_msg.orientation_covariance = data.orientation_covariance
    imu_msg.angular_velocity = data.angular_velocity
    imu_repub.publish(imu_msg)


if __name__=='__main__':
    rospy.init_node('imu_republisher', anonymous=True)
    rospy.Subscriber('/imu/data',Imu,imuDataReceived)
    imu_repub = rospy.Publisher('/imu/remapped',Imu)
    rospy.spin()
