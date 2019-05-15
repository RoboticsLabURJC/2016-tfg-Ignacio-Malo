#!/usr/bin/env python
'''import sys
import copy
import actionlib
import rospy
from moveit_python import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from control_msgs.msg import PointHeadAction, PointHeadGoal
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from std_msgs.msg import String
import cv2
import numpy as np
from math import pi'''
#import rospy
from first_prototype_tests_template_ready import *
'''EL objetivo es hacer 3 APIS:
Interfaz de manejo de articulaciones
look_at_object -> Mover la cabeza OK

Interfaz de acceso a camara 
convert_to_cv2 OK
save_image OK
convert_to_hsv OK
detect_objects OK

InterfazMovimiento
set_orientation_constraints -> definir constantes de movimiento

PlanningScene(externa)
addCylinder
addBox
MoveGroupCommander(externa)
set_plannning_time
set_num_planning_attemps'''
#global detected_color

def image_callback(msg):
    print(cv2.__version__)
    print("Received an image!")    
    # Convert your ROS Image message to OpenCV2
    cv2_img = convert_to_cv2(msg)
    save_image(cv2_img)
    global count
    count=count+1
    if count==1:
        print("ya tenemos la imagen")
        hsv_img = convert_to_hsv(cv2_img)
        save_image(hsv_img)
        H_max_R = 5
        H_min_R = 0
        H_min_B = 110
        H_max_B = 130
        H_min_G = 40
        H_max_G = 70
        S_max = 255
        S_min = 50
        V_max = 255
        V_min = 50
        #Define min and max values to use in the mask. 
        lower_red= np.array([H_min_R,S_min,V_min])
        upper_red= np.array([H_max_R,S_max,V_max])
        lower_blue = np.array([H_min_B,S_min,V_min])
        upper_blue = np.array([H_max_B,S_max,V_max])
        lower_green = np.array([H_min_G,S_min,V_min])
        upper_green = np.array([H_max_G,S_max,V_max])
        #global detected_color
        objects_detected={"red":0,"green":0,"blue":0} #RGB
        detected_color=detect_objects(cv2_img, lower_red, upper_red, "red")
        objects_detected["red"]=detected_color
        detected_color=detect_objects(cv2_img, lower_green, upper_green, "green")
        objects_detected["green"]=detected_color
        detected_color=detect_objects(cv2_img, lower_blue, upper_blue, "blue")
        objects_detected["blue"]=detected_color
        start_planning()
        set_orientation_constraints("l_wrist_roll_link",1.0,0.1,0.1,0.1)
        print(objects_detected)
        for x in objects_detected:
            if objects_detected[x]==1:
                print("move to goal %s" % x)

                move_to_goal(x)
            else:
                print("%s object won't be pushed" % x)

        '''if detected_color == "red":
            objects_detected[0]=1
        else:
            detected_color=detect_objects(cv2_img, lower_green, upper_green, "green")
        if detected_color =="green":
            objects_detected[1]=1
        else:
            detected_color=detect_objects(cv2_img, lower_blue, upper_blue, "blue")
        if detected_color =="blue":
            objects_detected[2]=1'''
        print(objects_detected)
        rospy.spin()
    else:
        print("nothing to do here")

def main():
    print("started")
    rospy.init_node('image_listener')
    #Set the target points to look
    x = 0.7
    y = -0.7
    z = 0.4
    #Move PR2 head
    look_at_object(x,y,z)


    #Define your image topic and subscriber. It will process images according to the defined threshold
    count=0
    image_topic = "/head_mount_kinect2/rgb/image_raw"
    # Set up your subscriber and define its callback. It will call detect_objects and print the detected color. 
    rospy.Subscriber(image_topic, Image, image_callback)
    print("suscribed")
    # Spin until ctrl + c
    rospy.spin()
    #rospy.signal_shutdown('end image processing')
if __name__=='__main__':
    try:
        main() 
    except rospy.ROSInterruptException:
        pass