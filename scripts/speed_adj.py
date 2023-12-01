#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

class Speed_adj(): 
    mlx = 1
    maz = 0.4
    n_s = Twist()
    def __init__(self): 

        rospy.init_node("speed_adj", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 

        rospy.Subscriber("cmd_vel", Twist, self.cb) 

        self.pub = rospy.Publisher("cmd_vel2", Twist, queue_size=1)

        rospy.spin() 



    def cb(self, msg):
        self.n_s.angular.z = msg.angular.z * self.maz
        self.pub.publish(self.n_s)
    
        
    def cleanup(self): 
        # #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.  
        print("Bye bye!!!")

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    Speed_adj() 
