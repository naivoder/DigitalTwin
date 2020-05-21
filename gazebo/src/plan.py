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
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

 
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



  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = "cone_link"
    p.pose=Pose(Point(0,0,0.6996),Quaternion(0,-1, 0, 0))
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

def main():
  try:
    print "============ Press `Enter` to begin by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    motion = MoveGroupPythonInterface()

    print "============ Press `Enter` to move robot to start position..."
    raw_input()
    motion.go_to_joint_state()

    print "============ Press `Enter` to execute movement using a pose goal..."
    raw_input()
    motion.go_to_pose_goal()

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

