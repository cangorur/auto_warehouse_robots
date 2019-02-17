#include <include/reservation_master/ReservationMaster.h>
#include "ros/ros.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "reservation_master");
	ros::NodeHandle nh;

	ReservationMaster reservationMaster;
	ROS_INFO("Reservation master ready!");

	ros::Rate r(10);
	while(ros::ok()) {
		reservationMaster.update();
		ros::spinOnce();
		r.sleep();
	}
}
