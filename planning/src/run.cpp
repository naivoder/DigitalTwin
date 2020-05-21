#include <plan.h>

int main(int argc, char **argv)
{

/*This file contains the switch information used to call for specific motions from the terminal
  window. The cases must be updated if new functions are added or new functionality is desired.*/

    //Define ROS instance and asynchronous spinner for time and update management
    ros::init(argc, argv, "ART_custom_interface");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    //This will instruct user on proper formatting in the terminal window if bad input received
    if(argc != 2)
    {
        ROS_INFO(" ");
        ROS_INFO("\tPROPER ENTRY FORMAT:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun art_planning plan n");
	ROS_INFO("\n\t\t\t~n = Desired Case (1-8)");
        return 1;
    }

    ART_PLANNING::art_planningClass art_plan;

    int selection = atoi(argv[1]);
    
    /*This switch defines which functions will be called when a user requests a numbered operation
      The functions can be changed, moved around, etc. depending on desired operation.*/
      
    switch(selection)
    {
    case 1:
        art_plan.clear_octomap();    
        art_plan.home_pose();
        art_plan.spawn_creel();
        ros::Duration(3).sleep();
        art_plan.peg_collision_object();
        art_plan.move_base();
        ros::Duration(3).sleep();
        art_plan.peg9_approach_pose();
        art_plan.turn_off_collision();
        art_plan.peg9_place_pose();
        art_plan.cartesian_reverse();
        art_plan.reset_values();
        art_plan.turn_on_collision();
        ros::Duration(2).sleep();
        art_plan.home_pose();
        art_plan.delete_creel();
        art_plan.clear_octomap();
        break;
    case 2:
        art_plan.peg_collision_object();
        break;
    case 3:
        art_plan.turn_on_collision();
        art_plan.home_pose();
        break;    
    case 4:
        art_plan.peg9_approach_pose();
        break; 
	case 5:
	    art_plan.turn_off_collision();
	    art_plan.peg9_place_pose();
		break;
	case 6:
	    art_plan.spawn_creel();
	    break;
    case 7:
        art_plan.delete_creel();
	    break;
    case 8:
        art_plan.clear_octomap();
	    break;   
	case 9:
	    art_plan.cartesian_reverse(); 
	    break;
	case 10:
	    art_plan.move_base();     
    }
    
    //After performing desired operation stop the spinner and allow terminal request next prompt.
    spinner.stop();
    return 0;
}
