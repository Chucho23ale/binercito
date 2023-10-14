#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Teleop():
    MAX_LINEAR = 1.01
    MAX_ANGULAR = MAX_LINEAR * 2 / 0.3462


    def __init__(self):
        rospy.init_node("Teleop")
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher("ard_motor", Twist, queue_size=1)

        self.my_msg = Twist()
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.pub.publish(self.my_msg)
            r.sleep()

    def callback(self, msg): 
        self.my_msg.linear.x = msg.axes[1] * self.MAX_LINEAR
        self.my_msg.angular.z = msg.axes[0] * self.MAX_ANGULAR

    def cleanup(self):
        self.my_msg.linear.x = 0
        self.my_msg.angular.z = 0
        self.pub.publish(self.my_msg)

if __name__:="__main__":
    Teleop()