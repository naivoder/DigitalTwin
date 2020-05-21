#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")

    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/art/catkin_ws/src/art_gazebo/Creel/model.sdf", "r") as f:
        product_xml = f.read()
    

    print("Spawning model:%s", "creel")
    item_pose   =   [(-3, 0,0), (0, 0, 0, 1)]
    spawn_model("creel", product_xml, "", item_pose, "world")
