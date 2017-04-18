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

## Current Authors: Joseph Shepey, Carol Jung
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_python
import moveit_msgs.msg
import tf.transformations
import scipy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, Vector3 
from std_msgs.msg import String
from pr2_gripper_grasp_planner_cluster.srv import GraspPose, GraspPoseRequest


def move_to_pose(pose, group, robot, dtp):

  ## Planning to a Pose goal
  print "============ Generating plan"
  #print pose
  pose_target = Pose() #geometry_msgs.msg.Pose()
  #pose_target.orientation.w =  pose.orientation.w # 1.0
  pose_target.orientation =  pose.orientation  
  pose_target.position.x = pose.position.x # 0
  pose_target.position.y = pose.position.y # .2
  pose_target.position.z = pose.position.z # .9
  group.set_pose_target(pose_target)
  group.set_max_velocity_scaling_factor(0.2)
  #group.set_max_acceleration_scaling_factor(0.1)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan = group.plan()

  print "============ Waiting while RVIZ displays pose..."
  rospy.sleep(2)

 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again).
  print "============ Visualizing pose"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  dtp.publish(display_trajectory);

  print "============ Waiting while pose is visualized (again)..."
  rospy.sleep(1)

  ## Moving to a pose goal
  print "Press y to move robot to goal state. Press any other key to exit."
  if raw_input() == 'y':
    group.go(wait=True)
    print("SUCCESS: Wrist is in position")


def get_selected_grasp():
  rospy.wait_for_service('selected_grasp')
  print "Ready to receive selected grasp"
  try:
      selected_grasp = rospy.ServiceProxy('selected_grasp', GraspPose)
      resp1 = selected_grasp()
      print "Received."
      return (resp1.pose, resp1.box_dims)  #Need to change this
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e


def compute_pregrasp(pose):
  DISTANCE_FROM_GRASP_POSE = -0.10 #back the pre-grasp up 10 cm from goal grasp pose
  pregrasp_pose = Pose() 
  pregrasp_pose.orientation =  pose.orientation  
  pregrasp_pose.position.x = pose.position.x # 0
  pregrasp_pose.position.y = pose.position.y # .2
  pregrasp_pose.position.z = pose.position.z # .9
  orientation = pose.orientation
  quat = [orientation.x, orientation.y, orientation.z, orientation.w]
  mat = tf.transformations.quaternion_matrix(quat)
  #print "mat = ", mat
  start = [pose.position.x, pose.position.y, pose.position.z]
  #print "original post=",start
  pregrasp_poseList = list(mat[:,0][0:3]*DISTANCE_FROM_GRASP_POSE + scipy.array(start))
  #print "pregrasp pose=", pregrasp_poseList
  pregrasp_pose.position.x = pregrasp_poseList[0]
  pregrasp_pose.position.y = pregrasp_poseList[1]
  pregrasp_pose.position.z = pregrasp_poseList[2]
  return pregrasp_pose

def attach_object(object_dimensions,pregrasp_pose, grasp_pose, scene, link, object_name, dis):
  #first determine axis from which gripper will grab
  #then attach the object specified by its dimensions and distance from link
  if not pregrasp_pose.position.z - grasp_pose.position.z == 0:
    scene.attachBox(object_name, object_dimensions.x, object_dimensions.y, object_dimensions.z, 0,0,object_dimensions.z/2+dis, link )
  elif not pregrasp_pose.position.x - grasp_pose.position.x == 0:
    scene.attachBox(object_name, object_dimensions.x, object_dimensions.y, object_dimensions.z, object_dimensions.x/2+dis,0,0, link )
  else:
    scene.attachBox(object_name, object_dimensions.x, object_dimensions.y, object_dimensions.z, 0,object_dimensions.y/2+dis,0, link )




def pick_and_place():

  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_wrist_to_pregrasp', anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_python.PlanningSceneInterface("base_link")

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("manipulator")
  #group = moveit_python.MoveGroupCommander("manipulator")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(1)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  #print "============ Robot Groups:"
  #print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  #print "============ Printing robot state"
  #print robot.get_current_state()
  print "============"

  ## Wait for test_pr2_gripper_grasp_planner_cluster to publish selected grasp
  #rospy.Subscriber('selected_grasp', Pose, selected_grasp_callback)
  #rospy.spin() # keeps python from exiting until this node is stopped

   

  grasp_pose, object_dimensions = get_selected_grasp()  #get best pre-grasp pose

  #print "object dimensions:"
  #print object_dimensions.x, object_dimensions.y, object_dimensions.z 


  pregrasp_pose=compute_pregrasp(grasp_pose) 

  scene.removeAttachedObject("mybox")
  move_to_pose(pregrasp_pose, group, robot, display_trajectory_publisher)  #move to 
  move_to_pose(grasp_pose, group, robot, display_trajectory_publisher)
  #close_gripper()  To Do
  attach_object(object_dimensions,pregrasp_pose, grasp_pose, scene, "tool0", "mybox", 0.01)
  #scene.attachBox("mybox", object_dimensions.x, object_dimensions.y, object_dimensions.z, 0,0,object_dimensions.z/2+0.03, "tool0" )
  print "grasp_pose", grasp_pose
  print "pregrasp_pose", pregrasp_pose
  move_to_pose(pregrasp_pose, group, robot, display_trajectory_publisher)

  #move_to_pose(new_pose, group, robot, display_trajectory_publisher)  #To Do

  #scene.removeAttachedObject("mybox")





  #Attach object to gripper 
  #p = moveit_python.PlanningSceneInterface("base_link")
  #p.addCube("my_cube", 0.1, 1, 0, 0.5)
  
 



  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    pick_and_place()
  except rospy.ROSInterruptException:
    pass
