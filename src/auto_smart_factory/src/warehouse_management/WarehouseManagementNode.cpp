#include <warehouse_management/WarehouseManagement.h>

int main(int argc, char **argv){

  	ros::init(argc, argv, "warehouse_management");
	ros::NodeHandle nh;

	WarehouseManagement warehouseManagement;
	warehouseManagement.start();
  	ROS_INFO("WarehouseManagement ready!"); 

	ros::Rate r(10); // 10 hz
	while(ros::ok()){
  		ros::spinOnce();
		r.sleep();
	}

  	return 0;
}
