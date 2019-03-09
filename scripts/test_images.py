#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
from math import pi

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print(cv2.__version__)
    print("Received an image!")
    #print(msg)
    try:
        print("try")
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('cv2_img',cv2_img)
        cv2.imwrite('camera_image_rgb.jpeg', cv2_img)
    except CvBridgeError, e:
        print(e)
    else:
        print("else")
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image_rgb.jpeg', cv2_img)
        color_HSV = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
        cv2.imwrite('camera_image_hsv.jpeg', color_HSV)
        H_max_R = 6.28*(180/(2*pi))
        H_min_R = 4.19*(180/(2*pi))
        H_min_G = 4.19*(180/(2*pi))
        H_max_G = 4.19*(180/(2*pi))
        H_min_B = 4.19*(180/(2*pi))
        H_max_B = 4.19*(180/(2*pi))
        S_max = 255
        S_min = 50
        V_max = 255
        V_min = 50
        
        lower_red= np.array([0,50,50])
        upper_red= np.array([5,255,255])
        
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        lower_green = np.array([40,50,50])
        upper_green = np.array([70,255,255])
        redMask = cv2.inRange(color_HSV, lower_red, upper_red)
        cv2.imwrite('camera_image_hsv_redMask.jpeg', redMask)
        #redMaskDenoised = cv2.fastNlMeansDenoising(redMask,None,100,7,21)
        resRed = cv2.bitwise_and(cv2_img,cv2_img, mask= redMask)
        cv2.imwrite('redResult.jpeg', redMask)
        im2,contoursR,hierarchy = cv2.findContours(redMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        contoursR = sorted(contoursR,key = cv2.contourArea,reverse = True)[:1] #devuelve el mas grande
        print(len(contoursR))
        if len(contoursR) > 0:
            areaR= cv2.contourArea(contoursR[0])
            print(areaR)
            print("hay algo rojo")
            if areaR > 500:
                print("es un objeto 1")
            else:
                print("no es un objeto 1")
        #print(redMask)
        # Threshold the HSV image to get only blue colors
        blueMask = cv2.inRange(color_HSV, lower_blue, upper_blue)
        cv2.imwrite('camera_image_hsv_blueMask.jpeg', blueMask)
        resBlue = cv2.bitwise_and(cv2_img,cv2_img, mask= blueMask)
        cv2.imwrite('blueResult.jpeg', resBlue)
        im2,contoursB,hierarchy  = cv2.findContours(blueMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        contoursB = sorted(contoursB,key = cv2.contourArea,reverse = True)[:1] #devuelve el mas grande
        print(len(contoursB))
        if len(contoursB) > 0:
            areaB= cv2.contourArea(contoursB[0])
            print(areaB)
            print("hay algo azul")
            if areaB > 500:
                print("es un objeto 1")
            else:
                print("no es un objeto 1")
        greenMask = cv2.inRange(color_HSV, lower_green, upper_green)
        cv2.imwrite('camera_image_hsv_greenMask.jpeg', greenMask)
        resGreen = cv2.bitwise_and(cv2_img,cv2_img, mask= greenMask)
        cv2.imwrite('greenResult.jpeg', resGreen)
        im2,contoursG,hierarchy  = cv2.findContours(greenMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contoursG = sorted(contoursG,key = cv2.contourArea,reverse = True)[:1] #devuelve los el mas grande
        print(len(contoursG))
        if len(contoursG) > 0:
            areaG= cv2.contourArea(contoursG[0])
            print(areaG)
            print("hay algo verde")
            if areaG > 500:
                print("es un objeto 1")
            else:
                print("no es un objeto 1")
        #print(len(contoursG[1]))
        #print(len(contoursG[2]))
def main():
    print("started")
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/head_mount_kinect2/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()         
