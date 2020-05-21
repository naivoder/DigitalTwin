#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#TEST IF PLANNING VALUES ARE WITHIN TOLERANCE
def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)
  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True

#SETUP PYTHON INTERFACE
class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()


#INITIALIZE MOVEIT COMMANDER 
    moveit_commander.roscpp_initialize(sys.argv)

#CREATE ROSPY NODE
    rospy.init_node('move_group_python_interface',
                    anonymous=True)

#INSTANTIATE A ROBOT COMMANDER OBJECT:
    robot = moveit_commander.RobotCommander()

#INTATIATE A PLANNING SCENE INTERFACE:
    scene = moveit_commander.PlanningSceneInterface()

#INSTANTIATE A MOVE-GROUP COMMANDER OBJECT:
    group_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

#CREATE DISPLAY TRAJECTORY PUBLISHER (RVIZ):
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    creel_id = 'creel'
    pose_creel = geometry_msgs.msg.PoseStamped()
    pose_creel.header.frame_id = robot.get_planning_frame()
    pose_creel.pose.position.x = 3.5
    pose_creel.pose.position.y = .7
    pose_creel.pose.orientation.x = 0
    pose_creel.pose.orientation.y = 0
    pose_creel.pose.orientation.z = .7

    scene.add_mesh(creel_id,pose_creel,'./meshes/Creel_Rack_1_newOrigin.stl')
    objects = scene.get_known_object_names()
    print "============ Known objects: %s" % objects
 
#PRINT REFERENCE FRAME:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

#PRINT END EFFECTOR:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

#PRINT LIST OF ALL ROBOT MOVE-GROUPS:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

#PRINT LIST OF CURRENT JOINT STATES:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

#MISCELLANIOUS VARIABLES
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

#SET HOME/START POSITION BY ASSIGNING JOINT POSITIONS
  def go_startPose(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0   	#BASE-SHOULDER
    joint_goal[1] = 0   	#LOWER ARM
    joint_goal[2] = pi/2   	#UPPER ARM
    joint_goal[3] = -pi/2   	#WRIST 1
    joint_goal[4] = 0   	#WRIST 2
    joint_goal[5] = 0   	#WRIST 3
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

#MAP ENVIRONMENT IN CONTROLLED SWEEP
  def go_mapWorld(self):
    group = self.group
    joint_goal1 = group.get_current_joint_values()
    joint_goal1[0] = pi 	#BASE-SHOULDER
    joint_goal1[1] = 0   	#LOWER ARM
    joint_goal1[2] = pi/2   	#UPPER ARM
    joint_goal1[3] = -pi/2   	#WRIST 1
    joint_goal1[4] = 0   	#WRIST 2
    joint_goal1[5] = 0   	#WRIST 3
    group.go(joint_goal1, wait=True)
    group.stop()
    joint_goal2 = group.get_current_joint_values()
    joint_goal2[0] = -pi/2  	#BASE-SHOULDER
    joint_goal2[1] = 0   	#LOWER ARM
    joint_goal2[2] = pi/2   	#UPPER ARM
    joint_goal2[3] = -pi/2   	#WRIST 1
    joint_goal2[4] = 0   	#WRIST 2
    joint_goal2[5] = 0   	#WRIST 3
    group.go(joint_goal2, wait=True)
    group.stop()
    joint_goal3 = group.get_current_joint_values()
    joint_goal3[0] = 0   	#BASE-SHOULDER
    joint_goal3[1] = 0   	#LOWER ARM
    joint_goal3[2] = pi/2   	#UPPER ARM
    joint_goal3[3] = -pi/2   	#WRIST 1
    joint_goal3[4] = 0   	#WRIST 2
    joint_goal3[5] = 0   	#WRIST 3
    group.go(joint_goal3, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal3, current_joints, 0.01)

#FORCE ROBOT TO CARTESIAN COORDINATES - N0 PLAN
  def go_toCoordinates(self):
    group = self.group 
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0 	#ROTATIONAL POSITION - 1=HOME
    pose_goal.position.x = 0.5		#X-COORDINATE
    pose_goal.position.y = 0.5	        #Y-COORDINATE
    pose_goal.position.z = 1.0		#Z-COORDINATE
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()		#PREVENTS RESIDUAL MOTION
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_eeAdjust(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[5] = pi
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

#PLAN PATH TO CARTESIAN COORDINATE
  def plan_cartesian_path(self, scale=1):
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.8  # First move - down (z)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    return plan, fraction

#EXECUTE SUCCESFULLY PLANNED PATH
  def execute_plan(self, plan):
    group = self.group
    group.execute(plan, wait=True)

def main():
  try:
    print "============ Press `Enter` to activate robot ..."
    raw_input()
    motion = MoveGroupPythonInterface()

    print "============ Press `Enter` to move robot to start position..."
    raw_input()
    motion.go_startPose()

    print "============ Press `Enter` to scan the surrounding environment..."
    raw_input()
    motion.go_mapWorld()

    print "============ Press `Enter` to move robot to desired coordinates..."
    raw_input()
    motion.go_toCoordinates()

    print "============ Press `Enter` to turn cone to desired orientation..."
    raw_input()
    motion.go_eeAdjust()

    print "============ Press `Enter` to plan movement using a Cartesian path..."
    raw_input()
    cartesian_plan, fraction = motion.plan_cartesian_path()

    print "============ Press `Enter` to execute the the planned path..."
    raw_input()
    motion.execute_plan(cartesian_plan)

    print "============ Movement complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

