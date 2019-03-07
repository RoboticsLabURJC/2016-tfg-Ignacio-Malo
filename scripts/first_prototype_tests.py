#!/usr/bin/env python
import sys
import copy
import rospy
from moveit_python import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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
    rospy.sleep(2)
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
    p.addCylinder("blueCylinder",0.3,0.03,0.6,0.2,0.5)
    ## Planning to a Pose goal
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector
    '''print "============ Running code from ~/ws_moveit"
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0 #hacia delante?
    pose_target.position.x = 0.34 # a la dcha ---> Esto es delante/atras. Debe ser 0.34 para mvimiento 1, >0.44 para el 2
    pose_target.position.y = 0.4 
    pose_target.position.z = 0.6 #altura, esta es OK para golpear arriba
    group_left.set_pose_target(pose_target)
    p = PlanningSceneInterface("base_link")
    rospy.sleep(1)
    plan1 = group_left.plan()
    rospy.sleep(5)
    group_left.execute(plan1)
    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)
    pose_target.position.x = 0.54 # a la dcha
    pose_target.position.y = 0.4
    pose_target.position.z = 0.6 #altura
    group_left.set_pose_target(pose_target)
    plan2 = group_left.plan()
    rospy.sleep(5)
    group_left.execute(plan2)
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)
    moveit_commander.roscpp_shutdown()
    print "============ STOPPING"
    '''
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
                    print("no es un objeto 1")
            rospy.spin()
        else:
            print("nothing to do here")


def move_to_goal(color):
    global group_left
    global p
    #global group_right
    print(color)
    if color=='blue':
        move_group_python_interface_tutorial()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0 #hacia delante?
        pose_target.position.x = 0.34 # a la dcha ---> Esto es delante/atras. Debe ser 0.34 para mvimiento 1, >0.44 para el 2
        pose_target.position.y = 0.2 
        pose_target.position.z = 0.6 #altura, esta es OK para golpear arriba
        group_left.set_pose_target(pose_target)
        p = PlanningSceneInterface("base_link")
        rospy.sleep(1)
        plan1 = group_left.plan()
        rospy.sleep(2)
        p.removeCollisionObject("blueCylinder")
        group_left.execute(plan1)
        rospy.sleep(2)
        pose_target.position.x = 0.54 # a la dcha
        pose_target.position.y = 0.2
        pose_target.position.z = 0.6 #altura
        group_left.set_pose_target(pose_target)
        plan2 = group_left.plan()
        rospy.sleep(2)
        group_left.execute(plan2)
        print "============ Waiting while RVIZ displays plan2..."
        rospy.sleep(2)
        moveit_commander.roscpp_shutdown()
    elif color=='green':
        move_group_python_interface_tutorial()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0 #hacia delante?
        pose_target.position.x = 0.34 # a la dcha ---> Esto es delante/atras. Debe ser 0.34 para mvimiento 1, >0.44 para el 2
        pose_target.position.y = 0.4 
        pose_target.position.z = 0.6 #altura, esta es OK para golpear arriba
    elif color=='red':
        move_group_python_interface_tutorial()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0 #hacia delante?
        pose_target.position.x = 0.34 # a la dcha ---> Esto es delante/atras. Debe ser 0.34 para mvimiento 1, >0.44 para el 2
        pose_target.position.y = 0.0 
        pose_target.position.z = 0.6 #altura, esta es OK para golpear arriba
    else:
        print('NO HAY OBJETOS DISPONIBLES')


def main():
    print("started")
    #rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
    print("init node ok")
    #move_group_python_interface_tutorial()
    rospy.init_node('image_listener')
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