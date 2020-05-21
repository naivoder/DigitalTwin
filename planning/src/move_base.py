#!/usr/bin/env python
import moveit_commander, moveit_msgs.msg
import sys, tf, rospy, time
from std_msgs.msg import Float64

print("Moving ART Into Position")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mobile_base')
rate = rospy.Rate(10)  # 10 Hz just used for tf waiting etc.

# Publisher for mobile base
pub_mobile_base = rospy.Publisher('mobile_base_fake_controller/command', Float64, queue_size=1)

# Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object.
# This object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

d = rospy.Duration(1)

for i in [0.0, 1.0]:
    pub_mobile_base.publish(i)
    print('Moving Mobile Base')
    rospy.sleep(d)
