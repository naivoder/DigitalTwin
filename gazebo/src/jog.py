#!/usr/bin/env python

import sys, tf, copy, rospy, math
import moveit_commander, moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

behavior_selection = sys.argv[1]
rospy.init_node('pnp01')


# Change the gripper position
if behavior_selection == 'g':    
    # Publishers for gripper prismatic joints
    pub_gripper_p = rospy.Publisher('gripper_fake_controller_p/command', Float64, queue_size=3)
    pub_gripper_d = rospy.Publisher('gripper_fake_controller_d/command', Float64, queue_size=3)
    rospy.sleep(3)
    
    gripper_dist = float(sys.argv[2])
    pub_gripper_p.publish(-gripper_dist)
    pub_gripper_d.publish(gripper_dist)
    rospy.sleep(1)

# Move the manipulator
else:
    # First, initialize moveit_commander and rospy.
    print "============ Starting pickAndPlace01 setup"
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    
    # Instantiate a PlanningSceneInterface object.
    # This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    
    # Instantiate a MoveGroupCommander object.
    # This object is an interface to one group of joints.
    group = moveit_commander.MoveGroupCommander("robot_arm")
    rospy.sleep(1)
    
    # Relative motion in Cartesian space
    if behavior_selection == 'c':
        rel_x = float(sys.argv[2])
        rel_y = float(sys.argv[3])
        rel_z = float(sys.argv[4])
        
        # Get current pose and print
        pose_c = group.get_current_pose().pose
        print "========== Current pose: ", pose_c
        
        # Linear motion to desired pose
        waypoints = []
        print(' ')
        print(pose_c)
        waypoints.append(copy.deepcopy(pose_c))
        pose_c.position.x += rel_x
        pose_c.position.y += rel_y
        pose_c.position.z += rel_z
        waypoints.append(copy.deepcopy(pose_c))
        print(' ')
        print(pose_c)
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.001, 0.0)
        print('Before')
        group.execute(plan)
        print('After')        
        # Relative motion in joint space
    elif behavior_selection == 'j':
        joint_num = int(sys.argv[2])
        rel_joint = float(sys.argv[3])
        
        joint_c = group.get_current_joint_values()
        print "========== Current joints: ", joint_c
        joint_c[joint_num - 1] += rel_joint
        print "========== Target joints: ", joint_c
        group.set_joint_value_target(joint_c)
        group.go(True)
        
    else:
        print "Need to specify c, j, or g for Cartesian, joint, or gripper motion"
        moveit_commander.roscpp_shutdown()
        sys.exit(0)
        

moveit_commander.roscpp_shutdown()
