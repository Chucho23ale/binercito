#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster 
from math import cos, sin

class Odom_tf(): 
    r = 0.0636
    l=0.34
    wr = 0.0
    wl = 0.0
    thetaant = 0.0
    xant = 0.0
    yant = 0.0
    dt = 0.05
    def __init__(self): 

        rospy.init_node("odom_tf", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 

        rospy.Subscriber("wr", Float64, self.wr_cb) 
        rospy.Subscriber("wl", Float64, self.wl_cb) 


        #********** INIT NODE **********### 
        self.rate = rospy.Rate(20)
        print("Node initialized 20hz")
        while not rospy.is_shutdown():
            thetap = self.r * (self.wr - self.wl)/self.l
            theta = self.thetaant + (thetap*self.dt)
            xp = (self.r*(self.wr+self.wl)/2.0)*cos(theta)
            yp = (self.r*(self.wr+self.wl)/2.0)*sin(theta)
            x = self.xant + (xp*self.dt)
            y = self.yant + (yp*self.dt)

            br = TransformBroadcaster()
            q = quaternion_from_euler(0, 0, theta)
            br.sendTransform((x,y,0), q, rospy.Time.now(), "base_link", "odom")

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
