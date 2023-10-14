#!/usr/bin/env python  

import rospy  

from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

# This class will subscribe to the /base_scan topic and print some data 

class ObjectFollower():  

    def __init__(self):  

        rospy.init_node("ObjectFollower", anonymous=True)  

        rospy.on_shutdown(self.cleanup)  

 

        ############    SUBSCRIBERS   #######################  

        rospy.Subscriber("scan", LaserScan, self.lidar_cb)  
        pub = rospy.Publisher("ard_motor", Twist, queue_size=1)

        ############ CONSTANTS AND VARIABLES ################  

        self.lidar = LaserScan() #The data from the lidar will be kept here. 
        self.robot_vel = Twist()
        kv = 1.0
        kw = 1.0
        v = 0.0
        w = 0.0

        r = rospy.Rate(20) #20Hz  

        print("Node initialized 20hz") 

         

        while not rospy.is_shutdown():  

            if self.lidar.ranges:
                closest_range = min(self.lidar.ranges)
                index = self.lidar.ranges.index(closest_range)
                angle = self.lidar.angle_min + index * self.lidar.angle_increment
                v=kv*closest_range
                w=kw*angle
                self.robot_vel.linear.x=v
                self.robot_vel.angular.z=w
                pub.publish(self.robot_vel)

            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle.  

     

    def lidar_cb(self, lidar_msg):  

        ## This function receives the lidar message and copies this message to a member of the class  

        self.lidar = lidar_msg 

 

         

         

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    

        print("I'm dying, bye bye!!!")  

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    ObjectFollower()