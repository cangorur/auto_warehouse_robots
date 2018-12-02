#include <warehouse_management/WarehouseManagement.h>

#include <tf/transform_datatypes.h>

WarehouseManagement::WarehouseManagement(){
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	occupancyMapPub = pn.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1, true);
	markerPub = pn.advertise<visualization_msgs::MarkerArray>("abstract_visualization", 1000);
	robotHeartbeatSub = n.subscribe("robot_heartbeats", 1000, 
			&WarehouseManagement::receiveHeartbeat, this);
	taskplannerStateSub = n.subscribe("task_planner/status", 10, 
			&WarehouseManagement::receiveTaskPlannerState, this);
	vizPublicationTimer = pn.createTimer(ros::Duration(5.0), 
			&WarehouseManagement::publishVisualization, this);
}

WarehouseManagement::~WarehouseManagement(){
}

void WarehouseManagement::start(){

	getConfigs();

	initPackageGenerator(warehouseConfig, packageConfigs);
	initStorageManagement(warehouseConfig, packageConfigs);
	initTaskPlanner(warehouseConfig, robotConfigs, packageConfigs);
	initRoadmapGenerator(warehouseConfig);

	// initialize agents/robots
	for (auto_smart_factory::Robot robot : warehouseConfig.robots) {
		for (auto_smart_factory::RobotConfiguration robotConfig : robotConfigs) {
			if (robotConfig.type_name == robot.type) {
				initAgent(robot.id, warehouseConfig, robotConfig);
				break;
			}
		}
	}

	// create static tray markers
	createTrayMarker();
}

void WarehouseManagement::getConfigs(){
	getWarehouseConfiguration();
	getRobotConfigurations();
	getPackageConfigurations();
}

bool WarehouseManagement::getWarehouseConfiguration(){
	std::string srv_name = "config_server/get_map_configuration";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::GetWarehouseConfig>
		(srv_name.c_str());
	auto_smart_factory::GetWarehouseConfig srv;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s success!", srv_name.c_str());
		warehouseConfig = srv.response.warehouse_configuration;
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
		return false;
	}
}

bool WarehouseManagement::getRobotConfigurations(){
	std::string srv_name = "config_server/get_robot_configurations";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::GetRobotConfigurations>
		(srv_name.c_str());
	auto_smart_factory::GetRobotConfigurations srv;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s success!", srv_name.c_str());
		robotConfigs = srv.response.configs;
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
		return false;
	}
}

bool WarehouseManagement::getPackageConfigurations() {
	std::string srv_name = "config_server/get_package_configurations";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::GetPackageConfigurations>
		(srv_name.c_str());
	auto_smart_factory::GetPackageConfigurations srv;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s success!", srv_name.c_str());
		packageConfigs = srv.response.configs;
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
		return false;
	}
}


bool WarehouseManagement::initRoadmapGenerator(
        auto_smart_factory::WarehouseConfiguration warehouse_configuration) {

     std::string srv_name = "roadmap_generator/trigger_roadmap_generator";
     ros::ServiceClient client = n.serviceClient<auto_smart_factory::triggerRoadmapGenerator> (srv_name.c_str());
     auto_smart_factory::triggerRoadmapGenerator srv;
     srv.request.warehouse_configuration = warehouse_configuration;
     ros::service::waitForService(srv_name.c_str());

     if (client.call(srv)) {
		ROS_INFO("[warehouse management]: %s | success: %s",
				srv_name.c_str(), ((bool)srv.response.success ? "true" : "false"));
		return true;
	} else {
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
	    return false;
	}
	}

/*
 * This is not used in this version of the code. We have roadmap generator, path planner, traffic planner
 * all separated as different nodes.
bool WarehouseManagement::initRoadmapPlanner(
		auto_smart_factory::WarehouseConfiguration warehouse_configuration, 
		std::vector<auto_smart_factory::RobotConfiguration> robot_configurations){
	std::string srv_name = "roadmap_planner/init";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::initRoadmapPlanner>
		(srv_name.c_str());
	auto_smart_factory::initRoadmapPlanner srv;
	srv.request.warehouse_configuration = warehouse_configuration;
	srv.request.robot_configurations = robot_configurations;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s | success: %s", 
				srv_name.c_str(), ((bool)srv.response.success ? "true" : "false"));
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
	    	return false;
	}
}
*/

bool WarehouseManagement::initTaskPlanner(
		auto_smart_factory::WarehouseConfiguration warehouse_configuration, 
		std::vector<auto_smart_factory::RobotConfiguration> robot_configurations, 
		std::vector<auto_smart_factory::PackageConfiguration> package_configurations){
	std::string srv_name = "task_planner/init";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::InitTaskPlanner>(srv_name.c_str());
	auto_smart_factory::InitTaskPlanner srv;
	srv.request.warehouse_configuration = warehouse_configuration;
	srv.request.robot_configurations = robot_configurations;
	srv.request.package_configurations = package_configurations;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s | success: %s", 
				srv_name.c_str(), ((bool)srv.response.success ? "true" : "false"));
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
	    	return false;
	}
}


bool WarehouseManagement::initAgent(std::string agent_id, 
		auto_smart_factory::WarehouseConfiguration warehouse_configuration, 
		auto_smart_factory::RobotConfiguration robot_configuration){
	std::string srv_name = agent_id + "/init";
  	ros::ServiceClient client = n.serviceClient<auto_smart_factory::InitAgent>(srv_name.c_str());
	auto_smart_factory::InitAgent srv;
	srv.request.warehouse_configuration = warehouse_configuration;
	srv.request.robot_configuration = robot_configuration;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s | success: %s", 
				srv_name.c_str(), ((bool)srv.response.success ? "true" : "false"));
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
	    	return false;
	}
}

bool WarehouseManagement::initPackageGenerator(
		auto_smart_factory::WarehouseConfiguration warehouse_configuration, 
		std::vector<auto_smart_factory::PackageConfiguration> package_configurations){
	std::string srv_name = "package_generator/init";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::InitPackageGenerator>
		(srv_name.c_str());
	auto_smart_factory::InitPackageGenerator srv;
	srv.request.warehouse_configuration = warehouse_configuration;
	srv.request.package_configurations = package_configurations;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s | success: %s", 
				srv_name.c_str(), ((bool)srv.response.success ? "true" : "false"));
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
	    	return false;
	}
}

bool WarehouseManagement::initStorageManagement(
		auto_smart_factory::WarehouseConfiguration warehouse_configuration, 
		std::vector<auto_smart_factory::PackageConfiguration> package_configurations){
	std::string srv_name = "storage_management/init";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::InitStorageManagement>
		(srv_name.c_str());
	auto_smart_factory::InitStorageManagement srv;
	srv.request.warehouse_configuration = warehouse_configuration;
	srv.request.package_configurations = package_configurations;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[warehouse management]: %s | success: %s", 
				srv_name.c_str(), ((bool)srv.response.success ? "true" : "false"));
		return true;
	}else{
		ROS_ERROR("[warehouse management]: Failed to call service %s!", srv_name.c_str());
	    	return false;
	}
}

void WarehouseManagement::publishVisualization(const ros::TimerEvent& e) {
	// publish occupancy map
	occupancyMapPub.publish(warehouseConfig.occupancy_map);

	// publish markers
	markerPub.publish(trayMarkers);
}

void WarehouseManagement::createTrayMarker() {
	trayMarkers.markers.clear();

	// setup tray marker
	visualization_msgs::Marker m;
	m.header.frame_id = "map";
	m.header.stamp = ros::Time::now();
	m.ns = "trays";
	m.id = 0;
	m.type = visualization_msgs::Marker::CUBE_LIST;
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = warehouseConfig.tray_geometry.width;
	m.scale.y = warehouseConfig.tray_geometry.height;
	m.scale.z = 0.1;
	m.lifetime = ros::Duration(0);
	m.frame_locked = true;

	for(const auto_smart_factory::Tray &tray : warehouseConfig.trays) {
		// tray marker
		geometry_msgs::Point p;
		p.x = tray.x;
		p.y = tray.y;
		p.z = 0.05;
		m.points.push_back(p);

		std_msgs::ColorRGBA col;
		col.a = 1;
		col.b = 0;
		if(tray.type == "input") {
			col.r = 0;
			col.g = 1;
			col.b = 0;
		}
		if(tray.type == "storage") {
			col.r = 1;
			col.g = 1;
			col.b = 0;
		}
		if(tray.type == "output") {
			col.r = 1;
			col.g = 0;
			col.b = 0;
		}
		if(tray.type == "charging station") {
			col.r = 0;
			col.g = 0;
			col.b = 1;
		}
		m.colors.push_back(col);

		// tray id text marker
		visualization_msgs::Marker idMarker;
		idMarker.header.frame_id = "map";
		idMarker.header.stamp = ros::Time::now();
		idMarker.ns = "tray_ids";
		idMarker.id = tray.id;
		idMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		idMarker.action = visualization_msgs::Marker::ADD;
		idMarker.scale.x = 0;
		idMarker.scale.y = 0;
		idMarker.scale.z = 0.3;
		idMarker.color.a = 1;
		idMarker.color.r = 1.0 - col.r;
		idMarker.color.g = 1.0 - col.g;
		idMarker.color.b = 1.0 - col.b;
		idMarker.lifetime = ros::Duration(0);
		idMarker.frame_locked = true;
		idMarker.text = std::to_string(tray.id);
		idMarker.pose.position.x = tray.x;
		idMarker.pose.position.y = tray.y;
		idMarker.pose.position.z = 0.3;

		trayMarkers.markers.push_back(idMarker);
	}
	trayMarkers.markers.push_back(m);
}

void WarehouseManagement::receiveHeartbeat(auto_smart_factory::RobotHeartbeat hb) {
	// get robot id (int)
	std::string idStr = hb.id.substr(hb.id.find("_") + 1);
	int id = atoi(idStr.c_str());
	float robotRadius = robotConfigs[0].radius;

	visualization_msgs::Marker m;
	m.header.frame_id = "map";
	m.header.stamp = ros::Time::now();
	m.ns = "robots_pose";
	m.id = id;
	m.type = visualization_msgs::Marker::ARROW;
	m.action = visualization_msgs::Marker::ADD;
	m.scale.x = robotRadius + 0.25;
	m.scale.y = 0.1;
	m.scale.z = 0.1;
	m.color = batteryLevelToColor(hb.battery_level);
	m.lifetime = ros::Duration(0);
	m.frame_locked = true;
	m.pose.position.x = hb.position.x;
	m.pose.position.y = hb.position.y;
	m.pose.position.z = hb.position.z;
	m.pose.orientation = tf::createQuaternionMsgFromYaw(hb.orientation);

	visualization_msgs::Marker m2;
	m2.header.frame_id = "map";
	m2.header.stamp = ros::Time::now();
	m2.ns = "robots_radius";
	m2.id = id;
	m2.type = visualization_msgs::Marker::SPHERE;
	m2.action = visualization_msgs::Marker::ADD;
	m2.scale.x = 2. * robotRadius;
	m2.scale.y = 2. * robotRadius;
	m2.scale.z = 2. * robotRadius;
	m2.color = batteryLevelToColor(hb.battery_level);
	m2.color.a = 1.;
	m2.lifetime = ros::Duration(0);
	m2.frame_locked = true;
	m2.pose.position.x = hb.position.x;
	m2.pose.position.y = hb.position.y;
	m2.pose.position.z = hb.position.z;
	m2.pose.orientation = tf::createQuaternionMsgFromYaw(hb.orientation);

	visualization_msgs::Marker label;
	label.header.frame_id = "map";
	label.header.stamp = ros::Time::now();
	label.ns = "robots_label";
	label.id = id;
	label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	label.action = visualization_msgs::Marker::ADD;
	label.scale.z = 0.3;
	label.color.a = 1;
	label.color.r = 1. - m.color.r;
	label.color.b = 1. - m.color.b;
	label.color.g = 1. - m.color.g;

	if(hb.idle) {
		label.text = std::to_string(id);
	} else {
		label.text = "[" + std::to_string(id) + "]";
	}

	label.lifetime = ros::Duration(0);
	label.frame_locked = true;
	label.pose.position.x = hb.position.x;
	label.pose.position.y = hb.position.y;
	label.pose.position.z = hb.position.z + 0.3;

	visualization_msgs::MarkerArray msg;
	msg.markers.push_back(m);
	msg.markers.push_back(m2);
	msg.markers.push_back(label);

	markerPub.publish(msg);
}

std_msgs::ColorRGBA WarehouseManagement::batteryLevelToColor(double batteryLevel) {
	std_msgs::ColorRGBA c;
	c.a = 1;
	if(batteryLevel >= 75.0) {
		c.r = 0;
		c.g = 2.0 - (batteryLevel - 50.0) / 25.0;
		c.b = 1;
	} else if(batteryLevel >= 50) {
		c.r = 0;
		c.g = 1;
		c.b = (batteryLevel - 50.0) / 25.0;
	} else if(batteryLevel >= 25.0) {
		c.r = 2.0 - batteryLevel / 25.0;
		c.g = 1;
		c.b = 0;
	} else if(batteryLevel > 0) {
		c.r = 1;
		c.g = batteryLevel / 25.0;
		c.b = 0;
	} else if(batteryLevel <= 0.0) {
		c.r = 0;
		c.g = 0;
		c.b = 0;
	}
	return c;
}

void WarehouseManagement::receiveTaskPlannerState(auto_smart_factory::TaskPlannerState msg) {
	ROS_INFO("---------- Current state of the task planner: ----------");

	ROS_INFO("Pending requests:");

	for (const auto_smart_factory::RequestStatus &req : msg.requests) {
		ROS_INFO("- [R %d] Type: %s    Status: %s", req.id, req.type.c_str(), req.status.c_str());
	}

	ROS_INFO("Current Tasks:");

	for (const auto_smart_factory::TaskState &task : msg.tasks) {
		ROS_INFO("- [T %d] Status: %s", task.id, task.status.c_str());
	}

	ROS_INFO("--------------------------------------------------------");
}
