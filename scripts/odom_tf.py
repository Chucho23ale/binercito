#!/usr/bin/env python3 
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import tf_conversions
import tf2_ros 
import math

class Odom_tf(): 
    r = 0.0636
    l=0.34
    wr = 0
    wl = 0
    thetaant = 0
    xant = 0
    yant = 0
    dt = 0.05
    def __init__(self): 
        rospy.init_node("Odom_tf", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 

        rospy.Subscriber("wr", Float64, self.wr_cb) 
        rospy.Subscriber("wl", Float64, self.wl_cb) 


        #********** INIT NODE **********### 
        self.rate = rospy.Rate(20)
        print("Node initialized 20hz")
        while not rospy.is_shutdown():
            thetap = self.r * (self.wr - self.wl)/self.l
            theta = self.thetaant + (thetap*self.dt)
            xp = (self.r(self.wr+self.wl)/2)*math.cos(theta)
            yp = (self.r(self.wr+self.wl)/2)*math.sin(theta)
            x = self.xant + (xp*self.dt)
            y = self.yant + (yp*self.dt)

            br = tf2_ros.TransformBroadcaster()
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)

            self.thetaant=theta
            self.xant = x
            self.yant = y
            self.rate.sleep()  



    def wr_cb(self, msg):
        self.wr = msg.data

    def wl_cb(self, msg):
        self.wl = msg.data
        
    def cleanup(self): 
        # #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.  
        print("Bye bye!!!")

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    Odom_tf() 