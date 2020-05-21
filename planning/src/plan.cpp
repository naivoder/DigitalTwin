#include <plan.h>

namespace ART_PLANNING
{

/*This file contains definitions for all custom functions contained in custom class*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::turn_off_collision()
        {
        //Define node handle (creates new nodes) and create a publisher to update the planning scene
        ros::NodeHandle node_handle;
        ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        
        //Define necessary class descriptions: robot model and planning scene
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene ps(kinematic_model);
        
        //Create an allowed collision matrix object using planning scene information
        collision_detection::AllowedCollisionMatrix acm = ps.getAllowedCollisionMatrix();
        
        //Define msgs to manipulate planning scene via the publisher created above
        moveit_msgs::AttachedCollisionObject collision_object;
        moveit_msgs::PlanningScene planning_scene; 	    
        
        //Add creel to allowed collision matrix, and set allowed collision to "true"
        acm.setEntry("peg",true);
        
        //Print allowed collision matrix to terminal window 
        acm.print(std::cout);
        
        //Send new allowed collision matrix to planning scene
        acm.getMessage(planning_scene.allowed_collision_matrix);
        
        //Update planning scene with new collision information
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
        ROS_INFO_STREAM("\033[1;35m\n---> PEG: ALLOWED COLLISIONS = TRUE\033[0m");   
        }
        
        /*This function will set the allowed collision matrix entry 'peg' to true, causing collisions to not occur*/
                
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::turn_on_collision()
        {
        //Define node handle (creates new nodes) and create a publisher to update the planning scene
        ros::NodeHandle node_handle;
        ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        
        //Define necessary class descriptions: robot model and planning scene
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene ps(kinematic_model);
        
        //Create an allowed collision matrix object using planning scene information
        collision_detection::AllowedCollisionMatrix acm = ps.getAllowedCollisionMatrix();
        
        //Define msgs to manipulate planning scene via the publisher created above
        moveit_msgs::AttachedCollisionObject collision_object;
        moveit_msgs::PlanningScene planning_scene; 	    
       
        //Add creel to allowed collision matrix, and set allowed collision to "true"
        acm.setEntry("peg",false);
       
        //Print allowed collision matrix to terminal window 
        acm.print(std::cout);
        
        //Send new allowed collision matrix to planning scene
        acm.getMessage(planning_scene.allowed_collision_matrix);
        
        //Update planning scene with new collision information
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
        ROS_INFO_STREAM("\033[1;35m\n---> PEG: ALLOWED COLLISIONS = FALSE\033[0m");   
        }
        
        /*This function will set the allowed collision matrix entry 'peg' to false, causing collisions to occur.*/
        
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::reset_values()
        {
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(1.0);    
        }
        
        /*This function ensures that any values changed during operation are reset and refreshed before 
          the next operation/plan is initiated. Called after cartesian path operation to reset velocity.*/     
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::home_pose()
	    {
	    //Publish info to terminal using exit codes to manipulate appearance 
	    ROS_INFO_STREAM("\033[1;33m\n---> GOING TO START POSITION\033[0m");
        
        //Get current joint values from the robot and store them as "current state"
        robot_state::RobotState current_state = *move_group.getCurrentState();
        
        //Define vector for desired joint values
        std::vector<double> joint_positions;
        
        //Define which group of joints this operation will affect, always the same for our simulation
        joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
        
        //Update "current state" to reflect desired position
        current_state.copyJointGroupPositions(joint_model_group, joint_positions);
	    joint_positions[0] = (0.00);
	    joint_positions[1] = (-2.36);
	    joint_positions[2] = (2.36);
	    joint_positions[3] = (1.57);
	    joint_positions[4] = (0.00);
	    joint_positions[5] = (0.00);   	
        
        //Test desired position for collisions, if position is not possible report with error message
        move_group.setJointValueTarget(joint_positions);
        bool success = (move_group.plan(the_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success)
            throw std::runtime_error("\033[1;31m\n--->UNABLE TO PLAN A REALISTIC PATH TO THIS POSE :(\033[0m");
        
        //If tested position is succesful, instruct the robot to move 
        move_group.move();
        ROS_INFO_STREAM("\033[1;32m\n---> SUCCESSFULLY EXECUTED TRAJECTORY\033[0m");
        ROS_INFO_STREAM("\033[1;36m\n---> READY FOR NEXT COMMAND\033[0m");      
	    }
	    
        /*This function moves the robot into a predefined "home" position listed as a set of joint values 
          seen above as a list of joint values. They can be changed with very little consequence. Largely
          used in simulation as an initial motion to update the octomap by causing the 3D camera
          connected to the base link to rotate around the world. To return to this position after cone
          placement be sure to reset collisions with creel to be "false" in allowed collision matrix.*/	    
	    	 	    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::peg_collision_object()
	    {
	    //Define node handle (creates new nodes) and create a publisher to update the planning scene
	    ros::NodeHandle node_handle;
        ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        
        //Define necessary class descriptions: robot model and planning scene
	    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene ps(kinematic_model);
        
        //Create an allowed collision matrix object using planning scene information
        collision_detection::AllowedCollisionMatrix acm = ps.getAllowedCollisionMatrix();
        
        //Define msgs to manipulate planning scene via the publisher created above
	    moveit_msgs::AttachedCollisionObject collision_object;
	    moveit_msgs::PlanningScene planning_scene; 
	      
	    //Begin creating collision object to replace octomap:
	    //--------------------------------------------------
	    
	    //Define which robot link the object will use as a frame of reference
	    collision_object.link_name = "base_link";
	    collision_object.object.header.frame_id = "base_link";
	    
	    //Create custom operation to add an object to the world
	    collision_object.object.operation = collision_object.object.ADD;
	    
	    //Name the new collision object
    	collision_object.object.id = "peg";
    	
    	//Eigen vector for scaling the mesh in three dimensions
    	Eigen::Vector3d scale(0.001, 0.001, 0.001);
    	
    	//Define file path to object mesh information
    	shapes::Mesh* m = shapes::createMeshFromResource("package://art_planning/Peg/meshes/peg.stl",scale);
    	
    	//Publish status to terminal while finishing shape operations
    	ROS_INFO_STREAM("\033[1;35m\n---> PEG COLLISION MESH ADDED TO WORLD\033[0m");
	    shape_msgs::Mesh mesh;
	    shapes::ShapeMsg mesh_msg;
	    shapes::constructMsgFromShape(m, mesh_msg);
	    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        
        //Define desired pose for collision object in simulation, must match actual creel pose
	    collision_object.object.meshes.resize(1);
	    collision_object.object.mesh_poses.resize(1);
	    collision_object.object.meshes[0] = mesh;
	    collision_object.object.header.frame_id = move_group.getPlanningFrame();
	    collision_object.object.mesh_poses[0].position.x = 0.9864+0.285;
	    collision_object.object.mesh_poses[0].position.y = 1.4868-0.15;
	    collision_object.object.mesh_poses[0].position.z = 0.270-0.055;
	    collision_object.object.mesh_poses[0].orientation.x = 0.4487197;
	    collision_object.object.mesh_poses[0].orientation.y = -0.5435769;
	    collision_object.object.mesh_poses[0].orientation.z = -0.5596877;
	    collision_object.object.mesh_poses[0].orientation.w = 0.4358032;
       
        //Add mesh object and pose information to the previously created custom msg
	    collision_object.object.meshes.push_back(mesh);
	    collision_object.object.mesh_poses.push_back(collision_object.object.mesh_poses[0]);
	  
	    //Send msg with new object to planning scene
	    std::vector<moveit_msgs::CollisionObject> collision_objects;
	    collision_objects.push_back(collision_object.object);
        
        //Update planning scene with new collision object
	    planning_scene.world.collision_objects.push_back(collision_object.object);
	    planning_scene.is_diff = true;
	    planning_scene_diff_publisher.publish(planning_scene);
	    ROS_INFO_STREAM("\033[1;36m\n---> READY FOR NEXT COMMAND\033[0m");	 
        }
    
        /*This function creates the peg as a "collision object", and adds that object to
          to the world. This overrides the octomap information and allows the user to turn collisions
          "on" and "off" at will.*/       
   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::peg9_approach_pose()
	    {
	    //Send action information to terminal window
	    ROS_INFO_STREAM("\033[1;33m\n---> GOING TO PEG #7 (90%) POSE\033[0m");
	    
	    //Using hardcoded end effector position defined in header file, test desired position
		move_group.setPoseTarget(peg9_approach);
		
		//If no possible path to desired position, send error message to terminal
		bool success = (move_group.plan(the_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if(!success)
			throw std::runtime_error("\033[1;31m\n:--->UNABLE TO PLAN A REALISTIC PATH TO THIS POSE :(");
        
        //If path is found, instruct robot to move following calculated trajectory
		move_group.move();
		ROS_INFO_STREAM("\033[1;32m\n---> SUCCESSFULLY EXECUTED TRAJECTORY\033[0m");
		ROS_INFO_STREAM("\033[1;36m\n---> READY FOR NEXT COMMAND\033[0m");
	    }

        /*This function moves the robot into what could be known as "the 90% pose", i.e. the robot will
          move the end effector to directly in front of the desired peg, to prepare for the cone to be 
          placed onto the creel in the next motion. Currently this movement uses the octomap created by
          the 3D camera to perform obstacle avoidance. This move could also be called after placement as
          a waypoint prior to returning to home position. The position is currently hard coded to match peg 6,
          however the function can be adapted to calculate position for the other 11 pegs if so desired.*/        
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::peg9_place_pose()
	    {
	    //Send action info to terminal window 
        ROS_INFO_STREAM("\033[1;33m\n---> COMPUTING CARTESIAN PATH\033[0m");
        
        //Define custom messages for trajectory
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose target_pose3 = peg9_approach;
        moveit_msgs::RobotTrajectory trajectory_msg;
        
        //Path for cone placement from column one approach poses
        target_pose3.position.z += 0.00;
        target_pose3.position.y -= 0.07;
        target_pose3.position.x += 0.3;
        waypoints.push_back(target_pose3);  

        //Initialize planning parameters
        move_group.setPlanningTime(10.0);
        double fraction = move_group.computeCartesianPath(waypoints,
                                                     0.01,  // eef_step
                                                     0.0,   // jump_threshold
                                                     trajectory_msg, false);
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "robot_arm");
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);         
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
 
        //Plan cartesian path, send status messages to terminal window         
        bool success = iptp.computeTimeStamps(rt);
        ROS_INFO("Computer time stamp = %s" ,success?"Succeded":"Failed");
        rt.getRobotTrajectoryMsg(trajectory_msg);
        the_plan.trajectory_ = trajectory_msg;
        ROS_INFO("Visualizing planned path (%.2f%% acheived)",fraction * 100.0);
        sleep(5.0);
        
        //Execute cartesian path, send status messages to terminal window
        move_group.execute(the_plan); 
        ROS_INFO_STREAM("\033[1;32m\n---> SUCCESSFULLY EXECUTED PATH\033[0m"); 
        ROS_INFO_STREAM("\033[1;36m\n---> CONE HUNG ON PEG #7\033[0m");  
        }
        
        /*This function is used to place the cone onto the desired peg. Must be called after the creel
          has collisions set to "true" in allowed collision matrix. Current path is hardcoded for one column only.
          To use for second column pegs the angle must be reversed. Solves/executes cartesian path.*/
                 	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::cartesian_reverse()
	    {
	    //Send action info to terminal window 
        ROS_INFO_STREAM("\033[1;33m\n---> COMPUTING CARTESIAN PATH\033[0m");
        
        //Define custom messages for trajectory
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
        moveit_msgs::RobotTrajectory trajectory_msg2;
        
        //Path for cone placement from column one approach poses
        target_pose.position.z += 0.00;
        target_pose.position.y += 0.075;
        target_pose.position.x -= 0.3;
        waypoints.push_back(target_pose);  

        //Initialize planning parameters
        move_group.setPlanningTime(10.0);
        double fraction = move_group.computeCartesianPath(waypoints,
                                                     0.01,  // eef_step
                                                     0.0,   // jump_threshold
                                                     trajectory_msg2, false);
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "robot_arm");
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg2);         
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
 
        //Plan cartesian path, send status messages to terminal window         
        bool success = iptp.computeTimeStamps(rt);
        ROS_INFO("Computer time stamp = %s" ,success?"Succeded":"Failed");
        rt.getRobotTrajectoryMsg(trajectory_msg2);
        the_plan.trajectory_ = trajectory_msg2;
        ROS_INFO("Visualizing planned path (%.2f%% acheived)",fraction * 100.0);
        sleep(5.0);
        
        //Execute cartesian path, send status messages to terminal window
        move_group.execute(the_plan); 
        ROS_INFO_STREAM("\033[1;32m\n---> SUCCESSFULLY EXECUTED PATH\033[0m");  
        }
        
        /*This function is the reverse of the peg placement cartesian approach, causing the manipulator to "back out"
          of the place position without changing its orientation. Should be called before returning to home position
          to avoid contact with peg collision mesh while collisions are disabled.*/
        
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::spawn_creel()
        {
        //Define node handle and create service for modifying creel pose
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        std::string fname = "/home/naivoder/catkin_ws/src/art_planning/src/transformation.py";
        std::string call = "python ";
        call += fname;
        system(call.c_str());
        }
        
        /*This function will spawn the creel according to peg position from robot perspective. Peg position
          is passed over zmq-socket. Auxillary python function transformation.py is executed by this function call.*/
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::delete_creel()
	    {  
	    //Call service and send status message to terminal window  
        system("rosservice call /gazebo/delete_model '{model_name: Creel}'");
	    }
	    
	    /*This function will delete the current creel model from the simulation.*/
	    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::clear_octomap()
	    {   
	    //Call service and send status message to terminal window 
        system("rosservice call /clear_octomap");
	    }
	    
	    /*This function will refresh the octomap, erasing memory of old creel locations but retaining current scene info.*/
	    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void art_planningClass::move_base()
	    {   
	    //Call service and send status message to terminal window 
        system("python /home/naivoder/catkin_ws/src/art_planning/src/move_base.py");
	    }
	    
}


