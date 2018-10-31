#include <agent/Gripper.h>
#include <agent/Agent.h>

Gripper::Gripper(Agent* _agent, ros::Publisher *gripper_state_pub) {
	agent=_agent;
	agentID = agent->getAgentID();
	gripperStatePub = gripper_state_pub;
}

Gripper::~Gripper(){
}

bool Gripper::loadPackage(bool load) {

	// get the postion from the agent
	geometry_msgs::Point robot_position = agent->getCurrentPosition();
	geometry_msgs::Quaternion robot_orientation = agent->getCurrentOrientation();
	tf::Quaternion q;
	tf::quaternionMsgToTF(robot_orientation, q);
	float orient = tf::getYaw(q);
	std::string str = load ? "load" : "unload";
	if (str == "load") {
		moveGripper(robot_position.x + (cos(orient) * 0.2), robot_position.y + (sin(orient) * 0.2), 0.38, package);
		ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>
				(agentID + "/gripper/" + str, this);
		std_srvs::Trigger srv;
		if (client.call(srv)) {
			if (srv.response.success) {
				// split MORSE response message to get to know the actual loaded package
				if (load) {
					std::vector<std::string> ids =
							split(split(srv.response.message, 'g')[1], '_');
					package.type_id = std::stoi(ids[0]);
					package.id = std::stoi(ids[1]);
				}
				ROS_INFO("[%s]: Succesfully %sed package: pkg%u_%u",
						 agentID.c_str(), str.c_str(), package.type_id, package.id);
				auto_smart_factory::GripperState gripper_state;
				gripper_state.loaded = load;
				gripper_state.package = package;
				gripperStatePub->publish(gripper_state);
				moveGripper(robot_position.x - (cos(orient) * 0.25), robot_position.y - (sin(orient) * 0.25), 0.35,
							package);
				return true;
			} else {
				ROS_ERROR("[%s]: Failed to %s package! %s",
						  agentID.c_str(), str.c_str(), srv.response.message.c_str());
			}

		} else
			ROS_ERROR("[%s]: Failed to call gripper service!", agentID.c_str());

		return false;
	} else if (str == "unload") {
		moveGripper(robot_position.x + (cos(orient) * 0.25), robot_position.y + (sin(orient) * 0.25), 0.38,
					package); //robot_position.y -/+ 0.25
	}
	ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>
			(agentID + "/gripper/" + str, this);
	std_srvs::Trigger srv;
	if (client.call(srv)) {
		if (srv.response.success) {
			// split MORSE response message to get to know the actual loaded package
			if (load) {
				std::vector<std::string> ids =
						split(split(srv.response.message, 'g')[1], '_');
				package.type_id = std::stoi(ids[0]);
				package.id = std::stoi(ids[1]);
			}
			ROS_INFO("[%s]: Succesfully %sed package : pkg%u_%u",
					 agentID.c_str(), str.c_str(), package.type_id, package.id);
			auto_smart_factory::GripperState gripper_state;
			gripper_state.loaded = false;
			gripper_state.package = package;
			gripperStatePub->publish(gripper_state);
			moveGripper(robot_position.x - (cos(orient) * 0.2), robot_position.y - (sin(orient) * 0.2), 0.38,
						package);
			return true;
		} else {
			ROS_ERROR("[%s]: Failed to %s package! %s",
					  agentID.c_str(), str.c_str(), srv.response.message.c_str());
		}

	} else
		ROS_ERROR("[%s]: Failed to call gripper service!", agentID.c_str());

	return false;
}

std::vector<std::string> Gripper::split(std::string &text, char sep) {
	std::vector<std::string> tokens;
	std::size_t start = 0, end = 0;
	while ((end = text.find(sep, start)) != std::string::npos) {
		tokens.push_back(text.substr(start, end - start));
		start = end + 1;
	}
	tokens.push_back(text.substr(start));
	return tokens;
}

bool Gripper::moveGripper(float x, float y, float z, auto_smart_factory::Package package) {
	std::string srv_name = "gripper_manipulator/move_gripper";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::MoveGripper>(srv_name.c_str());
	auto_smart_factory::MoveGripper srv;
	std::string grippr_id = agentID +".gripper";
	srv.request.gripper_id = grippr_id.c_str();
	srv.request.x = x;
	srv.request.y = y;
	srv.request.z = z;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)) {
		if (srv.response.success) {
			return true;
		}
	} else{
		ROS_ERROR("[gripper generator] Failed to call service %s %s",
				  srv_name.c_str(), grippr_id.c_str());
	}
	return false;
}
