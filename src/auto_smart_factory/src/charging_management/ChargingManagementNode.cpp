#include <charging_management/ChargingManagement.h>

int main(int argc, char **argv) {
  	ros::init(argc, argv, "charging_management");
	ros::NodeHandle nh;

	ChargingManagement chargingManagement;

  	ROS_INFO("ChargingManagement ready!"); 

	ros::Rate r(1); // 1 hz
	while(ros::ok()) {
		chargingManagement.update();
  		ros::spinOnce();
		r.sleep();
	}

  	return 0;
}
