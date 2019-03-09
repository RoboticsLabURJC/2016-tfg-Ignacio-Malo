#!/usr/bin/env python

# Modified by Ignacio Malo

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
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm. Object for right arm is also instantiated. 
  group = moveit_commander.MoveGroupCommander("left_arm")
  group_right = moveit_commander.MoveGroupCommander("right_arm")

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

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ End effector: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Running code from ~/ws_moveit"
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = 1.0
  pose_target.position.x = 0.7
  pose_target.position.y = -0.05
  pose_target.position.z = 1.1
  group.set_pose_target(pose_target)

  ##geometry_msgs::Pose target_pose1; C++ code
  ##target_pose1.orientation.w = 1.0;
  ##target_pose1.position.x = 0.28;
  ##target_pose1.position.y = -0.7;
  ##target_pose1.position.z = 1.0;
  ##move_group.setPoseTarget(target_pose1);

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()
  

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while plan1 is visualized (again)..."
  rospy.sleep(5)


  ## Moving to a pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Moving to a pose goal is similar to the step above
  ## except we now use the go() function. Note that
  ## the pose goal we had set earlier is still active 
  ## and so the robot will try to move to that goal. We will
  ## not use that function in this tutorial since it is 
  ## a blocking function and requires a controller to be active
  ## and report success on execution of a trajectory.

  # Uncomment below line when working with a real robot
  # group.go(wait=True)
  
  # Use execute instead if you would like the robot to follow 
  # the plan that has already been computed
  # group.execute(plan1)

  ## Planning to a joint-space goal 
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.

  group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan
  group_variable_values[0] = 1.0
  group.set_joint_value_target(group_variable_values)

  plan2 = group.plan()

  print "============ Waiting while RVIZ displays plan2..."
  rospy.sleep(5)


  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints 
  ## for the end-effector to go through.
  waypoints = []

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()  
  wpose.orientation.w = 1.0
  wpose.position.x += 0.1
  waypoints.append(copy.deepcopy(wpose))


  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               
  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)

  # Uncomment the line below to execute this plan on a real robot.
  # group.execute(plan3)
  
  '''
  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  # // Define a collision object ROS message. C++
  # moveit_msgs::CollisionObject collision_object;
  # collision_object.header.frame_id = move_group.getPlanningFrame();
  # // The id of the object is used to identify it.
  # collision_object.id = "box1";
  collision_object = moveit_msgs.msg.CollisionObject()
  collision_object.header.frame_id = group.get_planning_frame()
  collision_object.id = "box1"; #N
  # // Define a box to add to the world.
  # shape_msgs::SolidPrimitive primitive;
  # primitive.type = primitive.BOX; 
  # primitive.dimensions.resize(3);
  # primitive.dimensions[0] = 0.4;
  # primitive.dimensions[1] = 0.1;
  # primitive.dimensions[2] = 0.4;
  primitive = shape_msgs.msg.SolidPrimitive()
  primitive.type = primitive.BOX
  primitive.dimensions = [None] * 3
  primitive.dimensions[0] = 0.4
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;
  # //Define a pose for the box (specified relative to frame_id)
  # geometry_msgs::Pose box_pose;
  # box_pose.orientation.w = 1.0;
  # box_pose.position.x = 0.6;
  # box_pose.position.y = -0.4;
  # box_pose.position.z = 1.2;
  box_pose = geometry_msgs.msg.Pose()
  box_pose.orientation.w = 1.0
  box_pose.position.x = 0.6
  box_pose.position.y = -0.4
  box_pose.position.z = 1.2 
  
  # collision_object.primitives.push_back(primitive);
  # collision_object.primitive_poses.push_back(box_pose);
  # collision_object.operation = collision_object.ADD;
  collision_object.primitives.append(primitive)
  collision_object.primitive_poses.append(box_pose)
  collision_object.operation = collision_object.ADD
  # std::vector<moveit_msgs::CollisionObject> collision_objects;
  # collision_objects.push_back(collision_object);
  collision_objects = []
  collision_objects.append(collision_object)
    # // Now, let's add the collision object into the world
  # ROS_INFO_NAMED("tutorial", "Add an object into the world");
  # planning_scene_interface.addCollisionObjects(collision_objects);
  print "Add an object into the world"
  scene.addCollisionObjects('box_test',box_pose,[0.4,0.1,0.4] )
  ## First, we will define the collision object message'''

    
  #rospy.init_node("moveit_py")
  # create a planning scene interface, provide name of root link
  p = PlanningSceneInterface("base_link")

  # add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
  #p.addBox("my_cube", 0.4, 0.1, 0.4, 0.6, -0.4, 1.2)
  p.addBox("my_cube", 0.4, 0.1, 0.4, 0.6, 0.4, 1.2)
  rospy.sleep(1)

  group.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values()
  print "============ Joint values: ", group_variable_values
  print "Avoiding objects in trajectory"
  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan
  group_variable_values[0] = 1.0
  group.set_joint_value_target(group_variable_values)
  plan2 = group.plan()
  rospy.sleep(15)
  p.removeCollisionObject("my_cube")
  # do something

  # remove the cube
  #p.removeCollisionObject("my_cube")



  

  '''// Dual-arm pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms.
  static const std::string PLANNING_GROUP2 = "arms";
  moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);

  // Define two separate pose goals, one for each end-effector. Note that
  // we are reusing the goal for the right arm above
  two_arms_move_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

  geometry_msgs::Pose target_pose4;
  target_pose4.orientation.w = 1.0;
  target_pose4.position.x = 0.7;
  target_pose4.position.y = 0.15;
  target_pose4.position.z = 1.0;

  two_arms_move_group.setPoseTarget(target_pose4, "l_wrist_roll_link");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroupInterface::Plan two_arms_plan;

  success = (two_arms_move_group.plan(two_arms_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (dual arm plan) %s", success ? "" : "FAILED");'''
  group_right.set_pose_target(pose_target)
  plan_dual = group_right.plan()
  ## END_TUTORIAL

    ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

