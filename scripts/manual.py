#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def callback(msg): 
    my_msg = Twist()
    my_msg.linear.x = msg.axes[1]
    my_msg.angular.z = msg.axes[0]
    pub.publish(my_msg)

if __name__:="__main__":
    global pub
    rospy.init_node("manual")
    pub = rospy.Publisher("ard_motor", Twist, queue_size=1)
    rospy.Subscriber("chatter", Joy, callback) 
    rospy.spin() 