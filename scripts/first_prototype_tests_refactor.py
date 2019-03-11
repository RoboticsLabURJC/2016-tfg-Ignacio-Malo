#!/usr/bin/env python
import sys
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
from math import pi

#Instantiate CvBridge
bridge = CvBridge()
count=0

def move_group_python_interface_tutorial():
    global group_left
    global group_right
    global p
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)


    ## Instantiate a RobotCommander object.  Interface to the robot.
    robot = moveit_commander.RobotCommander()
    ## Instantiate a PlanningSceneInterface object.  Interface to the world.
    scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a MoveGroupCommander object.  Interface to groups of joints.
    group_left = moveit_commander.MoveGroupCommander("left_arm")
    #group_right = moveit_commander.MoveGroupCommander("right_arm")
    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)
    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    #rospy.sleep(2)
    print "============ Starting tutorial "
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group_left.get_planning_frame()
    ## We can also print the name of the end-effector link for this group
    print "============ End effector: %s" % group_left.get_end_effector_link()
    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()
    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    p = PlanningSceneInterface("base_link")
    p.addBox("table",0.2 ,1.25 ,0.55 ,0.6 ,0.775 ,0.275)
    p.addCylinder("redCylinder",0.3,0.03,0.6,0.7,0.7)
    p.addCylinder("blueCylinder",0.3,0.03,0.6,0.8,0.7)
    p.addCylinder("greenCylinder",0.3,0.03,0.6,0.9,0.7)
    p.addBox("obstacle1_1", 0.2, 0.1, 0.3, 0.6, 0.4, 0.7)#llega hasta 0.85
    p.addBox("obstacle1_2", 0.25, 0.1, 0.70, 0.35, 0.4, 0.35)
    p.addBox("obstacle1_3", 0.7, 0.1, 0.3, 0.6, 0.4, 1.35)
    #p.addBox("obstacle1_11", 0.2, 0.1, 0.65, 0.6, 0.4, 1.175)
    #p.addBox("obstacle1_21", 0.25, 0.1, 0.65, 0.35, 0.4, 1.275)
    '''#PRUEBA 1
    p.addBox("obstacle1_1", 0.2, 0.1, 0.25, 0.6, 0.4, 0.675)
    p.addBox("obstacle1_2", 0.25, 0.1, 0.70, 0.35, 0.4, 0.35)'''

def detect_objects(img, lower, upper, color):
    print(color)
    color_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(color_HSV, lower, upper)
    cv2.imwrite('refactorpy'+color+'.jpeg', mask)
    res = cv2.bitwise_and(img,img, mask= mask)
    #cv2.imwrite('redResult.jpeg', redMask)
    im2,contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours,key = cv2.contourArea,reverse = True)[:1] #devuelve el mas grande
    #prints(len(contoursR))
    if len(contours) > 0:
                area= cv2.contourArea(contours[0])
                print(area)
                print("hay algo %s"% color)
                if area > 500:
                    print("es un objeto %s"% color)
                    move_to_goal(color)
                else:
                    print("no es un objeto %s"% color)
def image_callback(msg):
    print(cv2.__version__)
    print("Received an image!")
    detected_color=""
    global count
    
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
        count=count+1
        if count==1:
            print("ya tenemos la imagen")
            color_HSV = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
            cv2.imwrite('camera_image_hsv.jpeg', color_HSV)
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
            detect_objects(cv2_img, lower_blue, upper_blue, "blue")
            detect_objects(cv2_img, lower_red, upper_red, "red")
            detect_objects(cv2_img, lower_green, upper_green, "green")
            '''redMask = cv2.inRange(color_HSV, lower_red, upper_red)
            cv2.imwrite('camera_image_hsv_redMask.jpeg', redMask)
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
                    detected_color="red"
                    move_to_goal(detected_color)
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
                    detected_color="blue"
                    move_to_goal(detected_color)
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
                    detected_color="green"
                    move_to_goal(detected_color)
                else:
                    print("no es un objeto 1")'''
            rospy.spin()
        else:
            print("nothing to do here")


def move_to_goal(color):
    global group_left
    global p
    y_red=0.7
    y_blue=0.8
    y_green=0.9
    #global group_right
    print(color)
    #p = PlanningSceneInterface("base_link")
    move_group_python_interface_tutorial()
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0 #hacia delante?
    pose_target.position.x = 0.34 # a la dcha ---> Esto es delante/atras. Debe ser 0.34 para mvimiento 1, >0.44 para el 2
    if color=='red':
        pose_target.position.y = y_red
    elif color =='blue':
        pose_target.position.y = y_blue
    elif color =='green':
        pose_target.position.y = y_green
    else:
        print("No destination found")
    pose_target.position.z = 0.8 #altura, esta es OK para golpear arriba
    group_left.set_pose_target(pose_target)
    plan1 = group_left.plan()
    group_left.execute(plan1)
    if color=='red':
        p.removeCollisionObject("redCylinder")
    elif color =='blue':
        p.removeCollisionObject("blueCylinder")
    elif color =='green':
        p.removeCollisionObject("greenCylinder")
    else:
        print("No objects to remove")
    rospy.sleep(1)
    #pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 1.0
    pose_target.position.x = pose_target.position.x + 0.1
    #pose_target.position.x = 0.44 
    #pose_target.position.y = 0.8
    #pose_target.position.z = 0.6 #altura
    print(pose_target.position)
    group_left.set_pose_target(pose_target)
    #plan2 = group_left.plan()
    #rospy.sleep(2)
    #group_left.execute(plan2)
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(2)
    moveit_commander.roscpp_shutdown()
    #PROvISIONAL

    '''pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0 #hacia delante?
    pose_target.position.x = 0.54 # a la dcha ---> Esto es delante/atras. Debe ser 0.34 para mvimiento 1, >0.44 para el 2
    pose_target.position.y = 0.4 
    pose_target.position.z = 0.85 #altura, esta es OK para golpear arriba
    group_left.set_pose_target(pose_target)
    
    #rospy.sleep(1)
    plan0 = group_left.plan()
    group_left.execute(plan0)'''
    
    #rospy.sleep(1)
    #move_group_python_interface_tutorial()
    #p = PlanningSceneInterface("base_link")
    
    #rospy.sleep(2)

    #rospy.sleep(2)
    #group_left.execute(plan1)
    #p.removeCollisionObject("obstacle1_1")
    #p.addBox("obstacle1_1_real", 0.25, 0.01, 0.7, 0.6, 0.5, 0.7)
    #p = PlanningSceneInterface("base_link")
    #rospy.sleep(1)
    #p.removeCollisionObject("blueCylinder")
    
    #p.addBox("obstacle1_2_bis", 0.25, 0.01, 0.3, 0.6, 0.55, 0.8)
    #rospy.sleep(2)

def look_at_bin():
    head_client = actionlib.SimpleActionClient("head_traj_controller/point_head_action",PointHeadAction)
    print("waiting")
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 0.7
    goal.target.point.y = -0.7
    goal.target.point.z = 0.4
    goal.min_duration = rospy.Duration(1.0)
    head_client.send_goal(goal)
    print(goal)
    head_client.wait_for_result()
    print("finished")

def main():
    print("started")
    #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    print("init node ok")
    #move_group_python_interface_tutorial()
    rospy.init_node('image_listener')
    look_at_bin()
    # Define your image topic
    image_topic = "/head_mount_kinect2/rgb/image_raw"
    # Set up your subscriber and define its callback
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