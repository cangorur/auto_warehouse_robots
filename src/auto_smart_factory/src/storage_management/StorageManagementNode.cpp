#include <ros/ros.h>
#include <storage_management/StorageManagement.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "storage_management");
	ros::NodeHandle nh;

	StorageManagement storageManagement;
	//ROS_INFO("StorageManagement ready!");

	ros::spin();
}
