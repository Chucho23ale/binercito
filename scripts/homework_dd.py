#!/usr/bin/env python  

import rospy  

from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist

# This class will subscribe to the /base_scan topic and print some data 

class Homework_dd():  

    def __init__(self):  

        rospy.init_node("ObjectFollower", anonymous=True)  

        rospy.on_shutdown(self.cleanup)  

 

        ############    SUBSCRIBERS   #######################  

        rospy.Subscriber("cmd", Int32, self.cmd_cb)  
        self.pub = rospy.Publisher("ard_motor", Twist, queue_size=1)

        ############ CONSTANTS AND VARIABLES ################  

        self.cmd = Int32() #The data from the lidar will be kept here. 
        self.robot_vel = Twist()

        r = rospy.Rate(50) #50Hz  

        print("Node initialized 50hz") 

         

        while not rospy.is_shutdown():  
            self.pub.publish(self.robot_vel)
            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle.  

     
    def timer_cb(self, timer_object):
        self.robot_vel.linear.x=0

    def cmd_cb(self, cmd_msg):  

        ## This function receives the lidar message and copies this message to a member of the class  

        if cmd_msg.data == 1:
            self.robot_vel.linear.x=0.25
            rospy.Timer(rospy.Duration(4), self.timer_cb, True)
        elif cmd_msg.data == 2:
            self.robot_vel.linear.x=0.25
            rospy.Timer(rospy.Duration(8), self.timer_cb, True)
        elif cmd_msg.data == 3:
            self.robot_vel.linear.x=0.5
            rospy.Timer(rospy.Duration(4), self.timer_cb, True)
        elif cmd_msg.data == 4:
            self.robot_vel.linear.x=0.5
            rospy.Timer(rospy.Duration(6), self.timer_cb, True)
        elif cmd_msg.data == 5:
            self.robot_vel.linear.x=1.0
            rospy.Timer(rospy.Duration(2), self.timer_cb, True)
        elif cmd_msg.data == 6:
            self.robot_vel.linear.x=1.0
            rospy.Timer(rospy.Duration(3), self.timer_cb, True)
        elif cmd_msg.data == 6:
            self.robot_vel.linear.x=1.0
            rospy.Timer(rospy.Duration(1), self.timer_cb, True)
 


    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    


        self.robot_vel.linear.x = 0
        self.robot_vel.angular.z = 0
        self.pub.publish(self.robot_vel)

        print("I'm dying, bye bye!!!")  

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    Homework_dd()