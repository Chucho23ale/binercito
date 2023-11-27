#!/usr/bin/env python  

import queue
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class recibirImgsPlantas():  

    def __init__(self):  

        rospy.init_node("plantas_subscriber", anonymous=True)  

        rospy.on_shutdown(self.cleanup)  

 

        ############    SUBSCRIBERS   #######################  

        rospy.Subscriber("video", Image, self.callback_video) 
        rospy.Subscriber("mask", Image, self.callback_mask)  
        rospy.Subscriber("contour", Image, self.callback_contour)  
        rospy.Subscriber("led", Bool, self.callback_led)   


        ############ CONSTANTS AND VARIABLES ################   
        self.video = Image()
        self.mask = Image()
        self.contour = Image()
        self.led = Bool()

        self.bridge = CvBridge()

        r = rospy.Rate(30) #30Hz  
         
        #Imprimir imagenes
        window_names = ["Imagen Original", "Máscara Verde", "Contornos"]

        # Inicializar las ventanas
        for name in window_names:
            cv2.namedWindow(name)

        while not rospy.is_shutdown():  
            if self.video.encoding:
                cv2.imshow(window_names[0], self.bridge.imgmsg_to_cv2(self.video))
                cv2.imshow(window_names[1], self.bridge.imgmsg_to_cv2(self.mask))
                cv2.imshow(window_names[2], self.bridge.imgmsg_to_cv2(self.contour))
                rospy.loginfo("Hay planta?? {}".format(self.led))
                key = cv2.waitKey(1)
                if key == 27:  # Espera 1ms, si se presiona la tecla 'Esc' (27 en ASCII), sale del bucle
                    break


            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle.  

     

    def callback_video(self, img):  
        self.video = img 
    
    def callback_mask(self, img):  
        self.mask = img 
    
    def callback_contour(self, img):  
        self.contour = img 
    
    def callback_led(self, led):  
        self.led = led 
         

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    

        print("I'm dying, bye bye!!!")  
        # Cerrar las ventanas al finalizar
        cv2.destroyAllWindows()

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    recibirImgsPlantas()  