#!/usr/bin/env python  

import rospy  

from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

# This class will subscribe to the /base_scan topic and print some data 

class ObstacleAvoidance():  

    field_of_view = 1
    d_stop = 1
    v = 0.3
    w = 0.5

    def __init__(self):  

        rospy.init_node("ObstacleAvoidance", anonymous=True)  

        rospy.on_shutdown(self.cleanup)  

 

        ############    SUBSCRIBERS   #######################  

        rospy.Subscriber("scan", LaserScan, self.lidar_cb)  
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        ############ CONSTANTS AND VARIABLES ################  

        self.lidar = LaserScan() #The data from the lidar will be kept here. 
        self.robot_vel = Twist()

        r = rospy.Rate(20) #20Hz  

        print("Node initialized 20hz") 

         

        while not rospy.is_shutdown():  

            if self.lidar.ranges:
                mid = len(self.lidar.ranges)/2
                adjustment = self.field_of_view/2/self.lidar.angle_increment
                new_ranges = self.lidar.ranges[int(mid-adjustment):int(mid+adjustment)]

                closest_range = min(new_ranges)

                if closest_range < self.d_stop:
                    self.robot_vel.linear.x=0
                    self.robot_vel.angular.z=self.w
                else:
                    self.robot_vel.linear.x=self.v
                    self.robot_vel.angular.z=0

                self.pub.publish(self.robot_vel)

        r.sleep()  #It is very important that the r.sleep function is called at least once every cycle.  

     

    def lidar_cb(self, lidar_msg):  

        ## This function receives the lidar message and copies this message to a member of the class  

        self.lidar = lidar_msg 

 

         

         

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    


        self.robot_vel.linear.x=0
        self.robot_vel.angular.z=0
        self.pub.publish(self.robot_vel)

        print("I'm dying, bye bye!!!")  

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    ObstacleAvoidance()