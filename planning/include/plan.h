#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include "geometric_shapes/shape_operations.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sstream>

namespace ART_PLANNING
{

//Defining custom class that will be used to plan and execute movements:

	class art_planningClass
	{
	  public:
		art_planningClass(): move_group(PLANNING_GROUP)
		{	 		      
	      peg9_approach.position.x = 0.483652;
	      peg9_approach.position.y = 0.379563 + 1.0;
	      peg9_approach.position.z = 0.248770;
	      peg9_approach.orientation.x = -0.553224;
	      peg9_approach.orientation.y = -0.450734;
	      peg9_approach.orientation.z = -0.544832;
	      peg9_approach.orientation.w = 0.4403860;
	      
	      move_group.allowReplanning(true);
	      move_group.setNumPlanningAttempts(10);
	      move_group.setPlanningTime(10);
		}
		
	    /*Hardcoded end effector positions/orientations for simulation. These are only relevant if creel
	      is spawned in proper location (definined in the art.world file), and will need to be adjusted
	      or solved for programatically if the creel will be changing positions.*/
		
        //List of all custom functions currently defined in this class:
        void spawn_creel();
		void delete_creel();
        void clear_octomap();
		void peg_collision_object();
		void turn_off_collision();
        void turn_on_collision();
		void home_pose();
	    void peg9_approach_pose();
		void peg9_place_pose();
		void cartesian_reverse();
		void reset_values();
		void move_base();

        //List of defined class objects, these should not be changed without significant reason:  
	  private:
		const std::string PLANNING_GROUP = "robot_arm";
		const robot_state::JointModelGroup* joint_model_group;
        moveit::planning_interface::PlanningSceneInterface virtual_world;
		moveit::planning_interface::MoveGroupInterface move_group;
		moveit::planning_interface::MoveGroupInterface::Plan the_plan;
        
        //Geometry msg for each hardcoded position should be added below: 
		geometry_msgs::Pose peg9_approach;
			
	};
}

