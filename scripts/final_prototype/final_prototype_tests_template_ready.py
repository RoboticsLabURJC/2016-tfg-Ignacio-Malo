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

def start_planning():
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
    p.addBox("table",0.2 ,1.25 ,0.55 ,0.65 ,0.775 ,0.275)
    p.addCylinder("redCylinder",0.3,0.03,0.65,0.7,0.7)
    p.addCylinder("blueCylinder",0.3,0.03,0.65,0.8,0.7)
    p.addCylinder("greenCylinder",0.3,0.03,0.65,0.9,0.7)
    p.addBox("obstacle1_1", 0.6, 0.2, 0.4, 0.65, 0.45, 0.95)
    p.addBox("obstacle1_2", 0.45, 0.01, 0.50, 0.3, 0.4, 0.3)
    print group_left.get_planning_time()
    group_left.set_planning_time(15)
    print group_left.get_planning_time()
    group_left.set_num_planning_attempts(20)

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
            return 1
            print("Se ha detectado un objeto de color %s" % (detected_color))
        else:
            print("no es un objeto %s"% color)
            return 0
    else:
        return 0
def convert_to_cv2(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        return cv2_img
    except CvBridgeError, e:
        print(e)

def convert_to_hsv(img):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return hsv_image

def save_image(img):
    cv2.imwrite('camera_image_rgb.jpeg', img)

def set_orientation_constraints(link, orientation, x_tol,y_tol,z_tol):
    ocm = moveit_msgs.msg.OrientationConstraint()
    ocm.link_name = link
    ocm.header.frame_id = "base_link"
    ocm.orientation.w = orientation
    ocm.absolute_x_axis_tolerance = x_tol
    ocm.absolute_y_axis_tolerance = y_tol
    ocm.absolute_z_axis_tolerance = z_tol
    ocm.weight = 1.0
    test_constraints= moveit_msgs.msg.Constraints()
    test_constraints.orientation_constraints.append(ocm)
    #ocm_forearm = moveit_msgs.msg.OrientationConstraint()
    print("constraints set")
def move_to_goal(color):
    global group_left
    global p
    y_red=0.7
    y_blue=0.8
    y_green=0.9
    #global group_right
    print(color)
    #p = PlanningSceneInterface("base_link")

    #start_planning()
    
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
    #group_left.execute(plan1)
    if color=='red':
        p.removeCollisionObject("redCylinder")
    elif color =='blue':
        p.removeCollisionObject("blueCylinder")
    elif color =='green':
        p.removeCollisionObject("greenCylinder")
    else:
        print("No objects to remove")
    '''rospy.sleep(1)
    pose_target.position.x = pose_target.position.x + 0.15
    print(pose_target.position)
    group_left.set_pose_target(pose_target)
    plan2 = group_left.plan()
    rospy.sleep(2)
    #group_left.execute(plan2)
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(2)'''
    moveit_commander.roscpp_shutdown()

def look_at_object(x,y,z):
    head_client = actionlib.SimpleActionClient("head_traj_controller/point_head_action",PointHeadAction)
    print("waiting")
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z
    goal.min_duration = rospy.Duration(1.0)
    head_client.send_goal(goal)
    print(goal)
    head_client.wait_for_result()
    print("finished")

def main():
    print("started")

if __name__=='__main__':
    try:
        main() 
    except rospy.ROSInterruptException:
        pass