#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
from moveit_python import *
import moveit_commander
from moveit_msgs.msg import *
import geometry_msgs.msg
import shape_msgs.msg


## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  rospy.init_node("moveit_py")

  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  #group_left = moveit_commander.MoveGroupCommander("left_arm")
  group = MoveGroupInterface("right_arm","odom_combined")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(10)
  print "============ Starting tutorial "

  rospy.init_node("moveit_py")

  g = PickPlaceInterface("right_arm", "right_eef") #This is the right end effector
  # create a planning scene interface, provide name of root link
  p = PlanningSceneInterface("base_footprint")
  p.removeCollisionObject("my_cube_1")
  #p.removeCollisionObject("my_cube_2")
  #p.removeCollisionObject("my_cube_2_1")
  p.removeCollisionObject("my_cube_3")
  print "Objetos borrados"
  rospy.sleep(3)
  # add some cubes in the base_footprint frame
  p.addBox("my_cube_1", 0.5, 1.5, 0.35, 0.7, -0.2, 0.175) #Desk
  #p.addBox("my_cube_2", 0.3, 0.1, 0.5, 0.7, -0.4, 0.5)  #Obstacle
  p.addBox("my_cube_2_1", 0.6, 0.1, 1.0, 0.7, -0.4, 1.0)  #Obstacle
  p.addBox("my_cube_3", 0.15, 0.01, 0.3, 0.6, -0.7, 0.5) #Object to pick
  #p.attachBox("my_cube_3", 0.15, 0.01, 0.3, 0.6, -0.7, 0.5, "r_wrist_roll_link") 
  print "Objetos en el mundo"
  rospy.sleep(1)

  grasps = moveit_msgs.msg.Grasp()
  pose_grasp_1 = geometry_msgs.msg.PoseStamped()
  pose_grasp_1.header.frame_id = "base_footprint";
  pose_grasp_1.pose.orientation.w = 1.0
  pose_grasp_1.pose.position.x = 0.34
  pose_grasp_1.pose.position.y = -0.7
  pose_grasp_1.pose.position.z = 0.5
  grasps.grasp_pose = pose_grasp_1

  grasps.pre_grasp_approach.direction.vector.x = 1.0;
  grasps.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  grasps.pre_grasp_approach.min_distance = 0.2;
  grasps.pre_grasp_approach.desired_distance = 0.4;

  grasps.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  grasps.post_grasp_retreat.direction.vector.z = 1.0;
  grasps.post_grasp_retreat.min_distance = 0.1;
  grasps.post_grasp_retreat.desired_distance = 0.25;

  #OPEN GRIPPER
  grasps.pre_grasp_posture.joint_names.append("r_gripper_joint");
  grasps.pre_grasp_posture.joint_names.append("r_gripper_motor_screw_joint");
  grasps.pre_grasp_posture.joint_names.append("r_gripper_l_finger_joint");
  grasps.pre_grasp_posture.joint_names.append("r_gripper_r_finger_joint");
  grasps.pre_grasp_posture.joint_names.append("r_gripper_r_finger_tip_joint");
  grasps.pre_grasp_posture.joint_names.append("r_gripper_l_finger_tip_joint");

  #grasps.pre_grasp_posture.points = [] #ojo a esta inicializacion
  
  points_1 = trajectory_msgs.msg.JointTrajectoryPoint()
  points_1.positions.append(1.0)
  points_2 = trajectory_msgs.msg.JointTrajectoryPoint()
  points_2.positions.append(0.477)
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_1);#TRY
  points_1.positions = [1.0]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_1);#TRY
  points_2.positions = [0.477]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  points_2.positions = [0.477]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  points_2.positions = [0.477]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  points_2.positions = [0.477]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  print grasps.pre_grasp_posture.points

  #CLOSE GRIPPER
  grasps.grasp_posture.joint_names.append("r_gripper_joint");
  grasps.grasp_posture.joint_names.append("r_gripper_motor_screw_joint");
  grasps.grasp_posture.joint_names.append("r_gripper_l_finger_joint");
  grasps.grasp_posture.joint_names.append("r_gripper_r_finger_joint");
  grasps.grasp_posture.joint_names.append("r_gripper_r_finger_tip_joint");
  grasps.grasp_posture.joint_names.append("r_gripper_l_finger_tip_joint");

  #grasps.pre_grasp_posture.points = [] #ojo a esta inicializacion
  
  points_1 = trajectory_msgs.msg.JointTrajectoryPoint()
  points_1.positions.append(0.0)
  points_2 = trajectory_msgs.msg.JointTrajectoryPoint()
  points_2.positions.append(0.2)
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_1);#TRY
  points_1.positions = [0.0]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_1);#TRY
  points_2.positions = [0.2]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  points_2.positions = [0.2]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  points_2.positions = [0.2]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  points_2.positions = [0.2]
  print grasps.pre_grasp_posture.points
  grasps.pre_grasp_posture.points.append(points_2);#TRY
  print grasps.pre_grasp_posture.points



  #g.setSupportSurfaceName("my_cube_1");
  g.pickup("my_cube_3", [grasps], support_name = 'my_cube_1');

  #PREPARANDO EL PLACE:

  loc = moveit_msgs.msg.PlaceLocation()
  pose_loc_1 = geometry_msgs.msg.PoseStamped()
  pose_loc_1.header.frame_id = "base_footprint";
  pose_loc_1.pose.orientation.w = 1.0
  pose_loc_1.pose.position.x = 0.7
  pose_loc_1.pose.position.y = 0.0
  pose_loc_1.pose.position.z = 0.5
  loc.place_pose = pose_loc_1

  loc.pre_place_approach.direction.vector.z = -1.0;
  loc.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
  loc.pre_place_approach.min_distance = 0.1;
  loc.pre_place_approach.desired_distance = 0.2;

  loc.post_place_retreat.direction.header.frame_id = "base_footprint";
  loc.post_place_retreat.direction.vector.x = -1.0;
  loc.post_place_retreat.min_distance = 0.1;
  loc.post_place_retreat.desired_distance = 0.25;

  #OPEN GRIPPER
  loc.post_place_posture.joint_names.append("r_gripper_joint");
  loc.post_place_posture.joint_names.append("r_gripper_motor_screw_joint");
  loc.post_place_posture.joint_names.append("r_gripper_l_finger_joint");
  loc.post_place_posture.joint_names.append("r_gripper_r_finger_joint");
  loc.post_place_posture.joint_names.append("r_gripper_r_finger_tip_joint");
  loc.post_place_posture.joint_names.append("r_gripper_l_finger_tip_joint");

  points_1 = trajectory_msgs.msg.JointTrajectoryPoint()
  points_1.positions.append(1.0)
  points_2 = trajectory_msgs.msg.JointTrajectoryPoint()
  points_2.positions.append(0.477)
  loc.post_place_posture.points.append(points_1);#TRY
  points_1.positions = [1.0]
  loc.post_place_posture.points.append(points_1);#TRY
  points_2.positions = [0.477]
  loc.post_place_posture.points.append(points_2);#TRY
  points_2.positions = [0.477]
  loc.post_place_posture.points.append(points_2);#TRY
  points_2.positions = [0.477]
  loc.post_place_posture.points.append(points_2);#TRY
  points_2.positions = [0.477]
  loc.post_place_posture.points.append(points_2);#TRY
  print loc.post_place_posture.points

  constr = moveit_msgs.msg.Constraints()
  ocm = moveit_msgs.msg.OrientationConstraint()
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = pose_loc_1.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = 3.14159265358979323846;
  ocm.weight = 1.0;
  constr.orientation_constraints.append(ocm)

  g.place("my_cube_3",[loc], support_name = 'my_cube_1', planner_id = 'RRTConnectkConfigDefault')

  rospy.sleep(10)
  '''
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  
  pickInterface= PickPlaceInterface("right_arm", "right_gripper")
  g1 = Grasp()
  g1.grasp_pose = pose_grasp_1

  g= []
  for i in range(10):
    g.append(g1)
  print g
  pickInterface.pick_with_retry("my_cube_3", g, support_name = "my_cube_1")

  place_pose_1 = geometry_msgs.msg.PoseStamped()
  place_pose_1.header.frame_id = "right_gripper";
  place_pose_1.pose.orientation.w = 1.0
  place_pose_1.pose.position.x = 0.34
  place_pose_1.pose.position.y = -0.6
  place_pose_1.pose.position.z = 0.5
  l1 = PlaceLocation()
  l1.place_pose = place_pose_1

  l= []
  for i in range(10):
    l.append(l1)
  print l
  # fill in l
  pickInterface.place_with_retry("my_cube_3", l, goal_is_eef = True, support_name = "my_cube_1")'''
  p.removeCollisionObject("my_cube_1")
  #p.removeCollisionObject("my_cube_2")
  p.removeCollisionObject("my_cube_2_1")
  p.removeCollisionObject("my_cube_3")
  # do something


  #group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  #group_variable_values = group.get_current_joint_values()

  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

