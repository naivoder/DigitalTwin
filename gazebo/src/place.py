#!/usr/bin/env python

import sys, copy, rospy, math, tf, numpy
from tf import transformations as t
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PoseStamped
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

print "============ Starting pickAndPlace01 setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pnp01')
rate = rospy.Rate(10)  # 10 Hz just used for tf waiting etc.

# TransformListener to get desired tool flange pose and methods
tf_listener = tf.TransformListener()
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
PlanningScene = moveit_commander.PlanningScene()
self._pubPlanningScene = rospy.Publisher('planning_scene', PlanningScene)
rospy.sleep(2)

creel_pose = PoseStamped()
creel_pose.header.frame_id = robot.get_planning_frame()
creel_pose.pose.position.x = -0.5
creel_pose.pose.position.y = 0.7
creel_pose.pose.position.z = 0.0
creel_pose.pose.orientation.x = 0.5
creel_pose.pose.orientation.y = 0.5
creel_pose.pose.orientation.z = 0.5
creel_pose.pose.orientation.w = 0.5
scene.add_mesh('creel1', creel_pose, './art_gazebo/Creel/meshes/creel.stl',size=(1, 1, 1))

rospy.wait_for_service('/get_planning_scene', 10.0)
get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
response = get_planning_scene(request)

acm = response.scene.allowed_collision_matrix
if not 'creel' in acm.default_entry_names:
    # add button to allowed collision matrix
    acm.default_entry_names += ['creel']
    acm.default_entry_values += [True]

    planning_scene_diff = PlanningScene(
            is_diff=True,
            allowed_collision_matrix=acm)

    self._pubPlanningScene.publish(planning_scene_diff)
    rospy.sleep(1.0)
