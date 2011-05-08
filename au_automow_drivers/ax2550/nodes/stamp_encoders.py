#!/usr/bin/env python
import roslib; roslib.load_manifest('ax2550')
import rospy
from ax2550.msg import StampedEncoders, Encoders

pub = None

def encodersCallback(data):
    global pub
    msg = StampedEncoders()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    msg.encoders = data
    pub.publish(msg)

def stamp_encoders():
    global pub
    rospy.init_node('stemp_encoders')
    pub = rospy.Publisher('encoders', StampedEncoders)
    sub = rospy.Subscriber('encoders_nostamp', Encoders, encodersCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        stamp_encoders()
    except rospy.ROSInterruptException: pass