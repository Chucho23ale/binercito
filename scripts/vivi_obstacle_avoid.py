#!/usr/bin/env python 
import rospy 
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# This class will subscribe to the /base_scan topic and print some data
class LaserSubClass(): 
    def __init__(self): 
        rospy.init_node("simple_obstacle_avoidance", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 

        ############    SUBSCRIBERS   ####################### 
        rospy.Subscriber("scan", LaserScan, self.lidar_cb) 

        ############    PUBLISHERS    #######################
        self.pub_cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=1)

        ############ CONSTANTS AND VARIABLES ################ 
        self.lidar = LaserScan() #The data from the lidar will be kept here.
        self.robot_vel = Twist()

        # Region de seguridad - d_stop
        self.d_stop = 0.5 # Cuando el objeto este a menos de 0.5 metros, para y gira

        # La frecuencia debe estar ligada a la velocidad de control del robot.
        # Esta el riesgo de que si es muy lento pueda chocar con cosas.
        self.r = rospy.Rate(25) #25Hz -- 50Hz si ves que la compu lo awanta
        #Si fuese un dron que ocupa moverse muy rapido, le subiriamos a como 100Hz
        print("Node initialized 25hz")
        
        while not rospy.is_shutdown(): # While(true) - infinito hasta que se cierre el nodo
            # Al menos una lectura valida del lidar
            if self.lidar.ranges: 
                closest_range = min(self.lidar.ranges) # Valor minimo
                index_closest = self.lidar.ranges.index(closest_range)
                angle = self.lidar.angle_min + self.lidar.angle_increment*index_closest
                # y = closest_range * sin (angle) - si es el opuesto
                #x = closest_range * cos (angle) - si es el adyacente

                # Rango de vision entre: pi/2 y -pi/2
                # Checo que el closest angle este dentro del rango de vision
                if angle > -1.57 and angle < 1.57:
                    # Si esta dentro del rango de vision, entonces
                    # checamos si closest_range esta dentro del area 
                    # de seguridad (d_stop)
                    if (closest_range <= self.d_stop):
                        # Para
                        self.robot_vel.linear.x   = 0
                        self.robot_vel.angular.z = 0
                        # Despues de parar, gira
                        self.robot_vel.angular.z = -1.0 #Turn to the right
                        self.robot_vel.linear.x = 0.0
                else:
                   # forward - TO the front
                    self.robot_vel.linear.x = 0.2 #Desired linear speed [m/s]
                    self.robot_vel.angular.z = 0.0

                # Lo publico a ROS - a Gazebo pa mover el robot
                self.pub_cmd_vel.publish(self.robot_vel)

            self.r.sleep()  #ESto regula que tan rapido se repite este loop 
    
    def lidar_cb(self, lidar_msg): 
        ## This function receives the lidar message and copies this message to a member of the class 
        self.lidar = lidar_msg

        
    def cleanup(self): 
        print("Stopping the robot")
        stop_twist = Twist()
        self.pub_cmd_vel.publish(stop_twist)  
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        print("Bye b*tch!!!")

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    LaserSubClass()