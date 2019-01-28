#include <agent/Agent.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, argv[1]);
	ros::NodeHandle nh;

	if(argc != 2) {
		//ROS_INFO("usage: Agent agent_id");
		return 1;
	}

	std::string agent_id = std::string(argv[1]);
	Agent agent(agent_id);

	//ROS_INFO("Agent %s ready!", agent_id.c_str());

	ros::Rate r(20); //20 hz
	while(ros::ok()) {
		agent.update();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
