#include <package_generator/PackageGenerator.h>

int main(int argc, char **argv) {
  	ros::init(argc, argv, "package_generator");
	ros::NodeHandle nh;

	PackageGenerator packageGenerator;

  	ROS_INFO("PackageGenerator ready!"); 

	ros::Rate r(1); //1 hz
	while(ros::ok()) {
		packageGenerator.update();
  		ros::spinOnce();
		r.sleep();
	}

  	return 0;
}
