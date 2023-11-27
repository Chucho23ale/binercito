#!/usr/bin/env python3 
import rospy
from std_msgs.msg import String 
#This class will subscribe to the /number topic and display the received number as a string message
class SeedClass(): 
    def __init__(self): 
        rospy.init_node("seed", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 

        ###******* INIT PUBLISHERS *******### 
        self.seed = rospy.Publisher('string_command', String,queue_size=1)

        ############ CONSTANTS AND VARIABLES ################ 
        # create a twist message, fill in the details
        self.seed_message = String() #Mensaje
        self.seed_message = "drop"

        #********** INIT NODE **********### 
        self.rate = rospy.Rate(0.05) #0.05Hz - cada 20seg
        print("Node initialized 0.05hz")
        while not rospy.is_shutdown():
            self.seed_message = "drop"
            self.seed.publish(self.seed_message)
            self.rate.sleep()  #It is very important that the r.sleep function is called at least onece every cycle.    

        
    def cleanup(self): 
        # #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.  
        print("Bye bye!!!")

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    SeedClass() 
