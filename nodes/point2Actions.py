#!/usr/bin/python

import rospy
from geometry_msgs.msg import Point
import roslib
roslib.load_manifest('nomad')

def callback(msg):
        print (msg.x, msg.y)

def listener():
        rospy.init_node('point2Actions')
        pub = rospy.Subscriber("nomad/points", Point, callback)
        
        rospy.spin()


if __name__ == '__main__':
        try:
                listener()
        except rospy.ROSInterruptException:
                pass
