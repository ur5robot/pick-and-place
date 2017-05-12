#!/usr/bin/python
'''
Test client for point_cluster_grasp_planner.  
Requires tabletop_node.launch to be running (in tabletop_object_detector).  
Add Markers of topic 'grasp_markers' to see the poses being tested.
'''

from __future__ import division
import roslib, rospy, actionlib
import numpy, scipy, time, sys, copy
import moveit_commander, moveit_python, moveit_msgs.msg

"""
#from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from pr2_gripper_grasp_planner_cluster.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
#from object_manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
#from object_manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
#from pr2_gripper_grasp_planner_cluster.srv import GraspPose, GraspPoseResponse
from visualization_msgs.msg import Marker
import tf.transformations
#import object_manipulator.draw_functions as draw_functions
import draw_functions as df
#from object_manipulator.convert_functions import *
import convert_functions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3 
from std_msgs.msg import String
#from pr2_gripper_grasp_planner_cluster.srv import GraspPose, GraspPoseRequest
"""


import sys
from grasp_planner.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
from grasp_planner.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from visualization_msgs.msg import Marker
import tf.transformations
#from grasp_planner.draw_functions import draw_functions as df

sys.path.insert(0, '../../grasp_planner/scripts')
import draw_functions as df
import convert_functions

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3 
from std_msgs.msg import String


## Convert a Pose message to a 4x4 scipy matrix
def pose_to_mat(pose):
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    pos = scipy.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    mat = scipy.matrix(tf.transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat

## Call the tabletop object detector to get the table and clusters in the scene
def call_object_detector():

    req = TabletopDetectionRequest()
    req.return_clusters = 1
    req.return_models = 0
    service_name = "/object_detection"
    rospy.loginfo("waiting for object_detection service")
    rospy.wait_for_service(service_name)
    serv = rospy.ServiceProxy(service_name, TabletopDetection)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling object_detection: %s"%e)  
        return 0
    return (res.detection.table, res.detection.clusters)


## Call the service to set the params for the cluster planner
def call_set_params(side_step = 0.02, palm_step = 0.005, overhead_grasps_only = False, side_grasps_only = False, include_high_point_grasps = True, pregrasp_just_outside_box = False, backoff_depth_steps = 5):
    
    req = SetPointClusterGraspParamsRequest()
    req.height_good_for_side_grasps = 0.05
    req.gripper_opening = 0.083
    req.side_step = side_step
    req.palm_step = palm_step
    req.overhead_grasps_only = overhead_grasps_only
    req.side_grasps_only = side_grasps_only
    req.include_high_point_grasps = include_high_point_grasps
    req.pregrasp_just_outside_box = pregrasp_just_outside_box
    req.backoff_depth_steps = backoff_depth_steps
    req.randomize_grasps = False
    rospy.loginfo("waiting for set params service")
    rospy.wait_for_service("set_point_cluster_grasp_params")
    serv = rospy.ServiceProxy("set_point_cluster_grasp_params", SetPointClusterGraspParams)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling set params: %s"%e)
        return 0
    return 1


## Call plan_point_cluster_grasp to get candidate grasps for a cluster
def call_plan_point_cluster_grasp(cluster):
    
    req = GraspPlanningRequest()
    req.target.reference_frame_id = "/base_link"
    req.target.cluster = cluster
    req.arm_name = "right_arm"
    req.collision_support_surface_name = "table"
    service_name = "plan_point_cluster_grasp"
    rospy.loginfo("waiting for plan_point_cluster_grasp service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, GraspPlanning)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling plan_point_cluster_grasp: %s"%e)  
        return 0
    if res.error_code.value != 0:
        return []
    
    return res.grasps


## Call plan_point_cluster_grasp_action to get candidate grasps for a cluster
def call_plan_point_cluster_grasp_action(cluster):
    
    goal = GraspPlanningGoal()
    goal.target.reference_frame_id = "/base_link"
    goal.target.cluster = cluster
    goal.arm_name = "right_arm"
    goal.collision_support_surface_name = "table"
    action_name = "plan_point_cluster_grasp"
    rospy.loginfo("waiting for plan_point_cluster_grasp action")
    client = actionlib.SimpleActionClient(action_name, GraspPlanningAction)
    client.wait_for_server()
    rospy.loginfo("action found")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    if not result or result.error_code.value != 0:
        return []
    return result.grasps


## Call evaluate_point_cluster_grasps to evaluate grasps on a cluster
def call_evaluate_point_cluster_grasps(cluster, grasps):

    req = GraspPlanningRequest()
    req.target.reference_frame_id = "/base_link"
    req.target.cluster = cluster
    req.arm_name = "right_arm"
    req.collision_support_surface_name = "table"
    req.grasps_to_evaluate = grasps
    service_name = "evaluate_point_cluster_grasps"
    rospy.loginfo("waiting for evaluate_point_cluster_grasps service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, GraspPlanning)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling evaluate_point_cluster_grasps: %s"%e)  
        return 0
    if res.error_code.value != 0:
        return []
    
    probs = [grasp.grasp_quality for grasp in res.grasps]
    return probs


## Call find_cluster_bounding_box to get the bounding box for a cluster
def call_find_cluster_bounding_box(cluster):
    
    req = FindClusterBoundingBoxRequest()
    req.cluster = cluster
    service_name = "find_cluster_bounding_box"
    rospy.loginfo("waiting for find_cluster_bounding_box service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)  
        return 0
    if not res.error_code:
        return (res.pose, res.box_dims)
    else:
        return (None, None)


## Detects object and computes grasp
def compute_grasp():

    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    draw_functions = df.DrawFunctions('grasp_markers')

    # set params for planner (change to try different settings)
    call_set_params(overhead_grasps_only = False, \
                    side_grasps_only = False, \
                    include_high_point_grasps = False, \
                    pregrasp_just_outside_box = False, \
                    backoff_depth_steps = 1)

    print "============ Detecting tabletop objects..."
    # call the tabletop object detector to detect clusters on the table
    (table, clusters) = call_object_detector()

    # keep going until the user says to quit (or until Ctrl-C is pressed)
    if (not rospy.is_shutdown()):

        # for each cluster in view in turn
        for cluster in clusters:
            draw_functions.clear_grasps(num=1000)

            print "============ Rendering bounding box for object..."
            # get the bounding box for this cluster and draw it in blue
            (box_pose, box_dims) = call_find_cluster_bounding_box(cluster)
            if box_pose == None:
                continue
            box_mat = pose_to_mat(box_pose.pose)
            box_ranges = [[-box_dims.x/2, -box_dims.y/2, -box_dims.z/2],
                          [box_dims.x/2, box_dims.y/2, box_dims.z/2]]
            draw_functions.draw_rviz_box(box_mat, box_ranges, 'base_link', \
                                         ns = 'bounding box', \
                                         color = [0,0,1], opaque = 0.25, duration = 60)
            draw_functions.clear_rviz_points('grasp_markers')

            print "============ Computing grasps for object"
            # call the grasp planner to compute grasps for bounding box
            grasps = call_plan_point_cluster_grasp(cluster)
            grasps = call_plan_point_cluster_grasp_action(cluster)
            grasp_poses = [grasp.grasp_pose for grasp in grasps]
            original_probs = [grasp.grasp_quality for grasp in grasps]

            # calculate success probability
            for grasp in grasps: # zero out original probabilities
                grasp.grasp_quality = 0
            probs = call_evaluate_point_cluster_grasps(cluster, grasps)
            print "grasp success probabilities:     ", probs
            print "number of grasps returned:", len(grasps)
            print "press p to visualize and select pre-grasp pose, or enter to select one with highest probability"
            
            
            print "============ Visualizing grasps"
            #draw the resulting grasps (all at once, or one at a time)
            print "enter to select grasp with highest probability, or press p to manually select grasp"
            best_grasp_pose = Pose()
            c = raw_input()
            if c == 'p':
                # visualize each grasp
                draw_functions.draw_grasps(grasp_poses, cluster.header.frame_id, pause = 1)
                print "enter the number of the desired pose corresponding to the order it was displayed"                
                while True:
                    print "i.e. press 0 for first grasp, 1 for second grasp, ..."
                    pose_id = input()
                    if pose_id >= 0 and pose_id < len(grasps):
                        break
                    print "invalid input. please select a number between 0 and %d" % len(grasps) - 1
                best_grasp_pose = grasps[pose_id].grasp_pose.pose

            else:
                #find the pose with highest success probability
                maxProbIndex = probs.index(max(probs))
                print "index of best grasp =", maxProbIndex
            
                #get the Pose of the best grasp
                best_grasp_pose = grasps[maxProbIndex].grasp_pose.pose
                print "displaying the selected grasp..."
                draw_functions.draw_grasps([grasps[maxProbIndex].grasp_pose], cluster.header.frame_id, pause = 0)

            pose = best_grasp_pose
            print "press enter to clear grasps and continue"
            raw_input()
            draw_functions.clear_grasps(num = 1000)

            return pose, box_dims
            #return GraspPoseResponse(best_grasp_pose, box_dims)


## Executes pick and place pipeline given a selected goal grasp pose
def pick_and_place(grasp_pose, object_dims):
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    # interface to the robot
    robot = moveit_commander.RobotCommander() 
    # interface to the world surrounding the robot.
    scene = moveit_python.PlanningSceneInterface("base_link")
    # interface to a group of joints. 
    group = moveit_commander.MoveGroupCommander("manipulator")
    # publisher to RVIZ for visualization
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(1)
    # name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()
    # name of the end-effector link for this group
    print "============ Reference frame: %s" % group.get_end_effector_link()


    ## MAIN ##
    pregrasp_pose = compute_pregrasp(grasp_pose) # compute pre-grasp
    move_to_pose(pregrasp_pose, group, robot, display_trajectory_publisher)  # move to pregrasp
    move_to_pose(grasp_pose, group, robot, display_trajectory_publisher)  # move to grasp

    """ To Do Close Gripper """
    #close_gripper()  To Do

    # attach object to ur5 in order to have successful motion planning
    attach_object(object_dims,pregrasp_pose, grasp_pose, scene, "tool0", "mybox", 0.01)
    move_to_pose(pregrasp_pose, group, robot, display_trajectory_publisher)

    # move ur5 to a neutral position
    up_pose = Pose()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    up_pose.orientation.x = quaternion[0]
    up_pose.orientation.y = quaternion[1]
    up_pose.orientation.z = quaternion[2]
    up_pose.orientation.w = quaternion[3]
    up_pose.position.x = 0
    up_pose.position.y = .2
    up_pose.position.z = 0.8
    print "MOVING TO NEUTRAL UP POSITION"
    move_to_pose(up_pose, group, robot, display_trajectory_publisher)

    # X and Y coordinates of where object should be placed
    PLACE_X = 0.5
    PLACE_Y = -0.3

    print "MOVING TO PRE RELEASE POSE"
    # even though we are releasing the object, still use grasp poses
    # change the x and y coordinates of the pregrasp pose
    pregrasp_pose.position.x = PLACE_X
    pregrasp_pose.position.y = PLACE_Y
    move_to_pose(pregrasp_pose, group, robot, display_trajectory_publisher)

    print "MOVING TO RELEASE POSE"
    # change the x and y coordinates of the grasp pose
    pregrasp_pose.position.z=pregrasp_pose.position.z-0.1
    move_to_pose(pregrasp_pose, group, robot, display_trajectory_publisher)

    """ To Do Open Gripper """
    # "remove" the object from the robot
    scene.removeAttachedObject("mybox")
    #open_gripper()  To Do

    print "MOVING TO NEUTRAL UP POSITION"
    # move back up to a neutral position awaiting next command
    move_to_pose(up_pose, group, robot, display_trajectory_publisher)
    print "SUCCESS"
    print "============ STOPPING"
    moveit_commander.roscpp_shutdown()


## Return a pregrasp pose 10cm backed along approach direction from goal grasp pose
def compute_pregrasp(pose):
    DISTANCE_FROM_GRASP_POSE = -0.10 
    pregrasp_pose = Pose() 
    pregrasp_pose.orientation =  pose.orientation  
    pregrasp_pose.position.x = pose.position.x 
    pregrasp_pose.position.y = pose.position.y 
    pregrasp_pose.position.z = pose.position.z 
    orientation = pose.orientation
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    mat = tf.transformations.quaternion_matrix(quat)
    start = [pose.position.x, pose.position.y, pose.position.z]
    pregrasp_poseList = list(mat[:,0][0:3]*DISTANCE_FROM_GRASP_POSE + scipy.array(start))
    pregrasp_pose.position.x = pregrasp_poseList[0]
    pregrasp_pose.position.y = pregrasp_poseList[1]
    pregrasp_pose.position.z = pregrasp_poseList[2]
    return pregrasp_pose


## Plans trajectory and moves robot to the specified pose
def move_to_pose(pose, group, robot, dtp):

    print "============ Generating plan"
    # create Pose object of goal pose
    pose_target = Pose() #geometry_msgs.msg.Pose()
    pose_target.orientation =  pose.orientation  
    pose_target.position.x = pose.position.x
    pose_target.position.y = pose.position.y
    pose_target.position.z = pose.position.z
    group.set_pose_target(pose_target)
    group.set_max_velocity_scaling_factor(0.3)
    #group.set_max_acceleration_scaling_factor(0.1)

    # plan to goal pose and visualize trajectory on RVIZ
    plan = group.plan()
    print "============ Visualizing pose on RVIZ"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    dtp.publish(display_trajectory);

    # Moving to goal pose
    print "Press y to move robot to goal state. Press any other key to exit."
    if raw_input() == 'y':
        group.go(wait=True)
        print("SUCCESS: Wrist is in position")


## Determine the axis of grasp orientation then 
## attach object based on its dimensions and distance from link
def attach_object(object_dimensions,pregrasp_pose, grasp_pose, scene, link, object_name, dis):
    if not pregrasp_pose.position.z - grasp_pose.position.z == 0:
        scene.attachBox(object_name, object_dimensions.x, object_dimensions.y, \
                        object_dimensions.z, 0, 0, object_dimensions.z / 2 + dis, link )

    elif not pregrasp_pose.position.x - grasp_pose.position.x == 0:
        scene.attachBox(object_name, object_dimensions.x, object_dimensions.y, \
                        object_dimensions.z, object_dimensions.x / 2 + dis, 0, 0, link )
    else:
        scene.attachBox(object_name, object_dimensions.x, object_dimensions.y, \
                        object_dimensions.z, 0,object_dimensions.y/2+dis,0, link )


if __name__ == "__main__":
    #initialize the node, tf listener and broadcaster, and rviz drawing helper class
    rospy.init_node('test_point_cluster_grasp_planner', anonymous=True)
    #Service() input parameters: service name, srv type name, request handler name
    try:
        pose, box_dims = compute_grasp()
        pick_and_place(pose, box_dims)
    except rospy.ROSInterruptException:
        pass