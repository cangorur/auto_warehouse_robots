#include <path_planning/PathPlanner.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle nh;

	PathPlanner pathplanner;
	
    ROS_INFO("Path planner is ready!");
	
	ros::spin();
}
