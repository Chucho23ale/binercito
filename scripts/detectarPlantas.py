#!/usr/bin/env python  

import queue
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

def buscarPlantas(img):
    #Boleano que dice si encontro una planta o no
    planta = False
    # Convertir la imagen a formato HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Aplicar un desenfoque para reducir el ruido
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Definir un rango para el color verde en HSV
    lower_green = np.array([30, 40, 40])  # Ajusta estos valores según tu necesidad
    upper_green = np.array([106, 255, 255]) #De 75 a150 en escala de 255

    # Crear una máscara para el rango verde
    mask = cv2.inRange(hsv, lower_green, upper_green)


    # Aplicar la máscara a la imagen original
    result = cv2.bitwise_and(img, img, mask=mask)

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Filtrar contornos por área
    min_plant_area = 1000  # Ajusta este valor según tus necesidades

    # Mostrar la imagen resultante en cada ventana
    #cv2.imshow(windows[0], img)
    #cv2.imshow(windows[1], result)
    # Dibujar los contornos en la imagen original
    for contour in contours:
    # Calcular el área del contorno
        area = cv2.contourArea(contour)
        # Filtrar contornos por área mínima
        if area > min_plant_area:
            planta = True
            # Dibujar el contorno en la imagen original
            cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
            # Obtener las coordenadas del rectángulo que rodea el contorno
            x, y, w, h = cv2.boundingRect(contour)

            # Agregar texto "planta" al lado del contorno
            cv2.putText(img, 'Planta', (x + w + 10, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    #cv2.imshow(windows[2], img)
    return planta, result, img

def main():

    rospy.init_node("DetectorPlantas", anonymous=True)  
    #Publishers
    publisherImg = rospy.Publisher("video", Image, queue_size=60)
    publisherMask = rospy.Publisher("mask", Image, queue_size=60)
    publisherContour = rospy.Publisher("contour", Image, queue_size=60)
    prenderLed = rospy.Publisher('led', Bool, queue_size=10)

    rate = rospy.Rate(30)

    videoCapture = cv2.VideoCapture(0)

    # Crear objeto CVBridge que es usado para convertir imagenes de opencv a las de ros
    bridge = CvBridge()

    while not rospy.is_shutdown():  
        # El primer valor es success o failure y el segundo la imagen en si
        returnValue, capFrame = videoCapture.read()
        if returnValue:
            #rospy.loginfo("Video frame captured and published")
            # Convertir imagen de opencv a objeto de ROS
            publisherImg.publish(bridge.cv2_to_imgmsg(capFrame))
            hayplanta, mascara, contour = buscarPlantas(capFrame)
            publisherMask.publish(bridge.cv2_to_imgmsg(mascara))
            publisherContour.publish(bridge.cv2_to_imgmsg(contour))
            prenderLed.publish(hayplanta)
            rospy.loginfo("Planta: {}".format(hayplanta))
            # cv2.imshow("camera", capFrame)
            

        rate.sleep()

if __name__ == "__main__":
    main()
