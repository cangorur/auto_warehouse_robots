#include <charging_management/ChargingManagement.h>

ChargingManagement::ChargingManagement() {
	ros::NodeHandle pn("~");
	this->initServer = pn.advertiseService("init", &ChargingManagement::init, this);
}

ChargingManagement::~ChargingManagement() {
}

bool ChargingManagement::init(auto_smart_factory::InitChargingManagement::Request &req, 
		auto_smart_factory::InitChargingManagement::Response &res) {
	if (!initialized) {
		initialized = initialize(req.warehouse_configuration);
		if (initialized)
			ROS_INFO("ChargingManagement succesfully initialized!");
		else
			ROS_ERROR("ChargingManagement initialization failed!");
	} else
		ROS_WARN("ChargingManagement has already been initialized!");
	res.success = initialized;
	return true;
}

bool ChargingManagement::initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration) {
	warehouseConfig = warehouse_configuration;
	initializeChargingStations();

	ros::NodeHandle pn("~");

	robotHeartbeatSub = pn.subscribe("/robot_heartbeats", 1000, 
			&ChargingManagement::receiveRobotHeartbeat, this);

	registerAgentChargingServer = pn.advertiseService("register_agent_charging_management",
	 		&ChargingManagement::registerAgent, this);
	getFreeChargingStationsServer = pn.advertiseService("get_free_charging_stations",
			&ChargingManagement::getFreeChargingStations, this);
	return true;
}

void ChargingManagement::initializeChargingStations() {
	for(int i=0; i < warehouseConfig.trays.size(); i++) {
		if(warehouseConfig.trays[i].type == "charging station")
			freeChargingStations.push_back(warehouseConfig.trays[i]);
	}
	ROS_INFO("[charging management] Manages %li charging stations!", freeChargingStations.size());
}

// Every one second it checks if the status of the robots registered for charging. Is it 
void ChargingManagement::update(){

		for(std::map< std::string, std::pair<auto_smart_factory::RobotConfiguration, std::string>>::iterator it = registeredRobots.begin(); 
			it!=registeredRobots.end(); ++it){
				//ROS_INFO("RObot %s has status of %s", it->first.c_str(), it->second.second.c_str());
		if(it->second.second == "unassigned"){
			int index = findBestStation(it->first);
			if(index>=0) {
				trackedRobots[it->first] = freeChargingStations[index].id;
				occupiedChargingStations[it->first] = freeChargingStations[index];
				freeChargingStations.erase(freeChargingStations.begin()+index);
				it->second.second = "assigned";
				ROS_INFO("Robot %s has been assigned to charging station (id: %d)", it->first.c_str(), trackedRobots[it->first]);
				ROS_INFO("Current available charging stations are %lu, occupied are %lu", freeChargingStations.size(), occupiedChargingStations.size());
				}
			else { 
				ROS_WARN("One failed attempt to assign charging station");
				}
			}
		if(it->second.second == "assigned"){
			if(ChargingManagement::assignRobot(it->first, trackedRobots[it->first],false))  // Assign robot with charging task
				it->second.second = "requested";
			else ROS_WARN("An charging task has been rejected by %s", it->first.c_str());
		}
		if(it->second.second == "charged"){
			if(ChargingManagement::assignRobot(it->first, trackedRobots[it->first], true)) {  // End robot charging task
				it->second.second = "untracked";
				freeChargingStations.push_back(occupiedChargingStations[it->first]);
				occupiedChargingStations.erase(it->first);
				ROS_INFO("%s has completed charging, currently %lu freeChargingStations, %lu occupied", it->first.c_str(), freeChargingStations.size(), occupiedChargingStations.size() );
			}
			else ROS_WARN("Fail to end charging task of %s", it->first.c_str());
		}
		if(it->second.second == "requested"){
		ROS_INFO("%s should be at charging station %d", it->first.c_str(), trackedRobots[it->first]);
		}
	}
}

bool ChargingManagement::registerAgent(auto_smart_factory::RegisterAgentChargingRequest &req,
	 	auto_smart_factory::RegisterAgentChargingResponse &res) {
	if (registeredRobots.count(req.agent_id) == 0) {
		// add new registered robot
		registeredRobots[req.agent_id].first = req.robot_configuration;
		registeredRobots[req.agent_id].second = "untracked";

		ROS_INFO("Registered agent at charging stations: %s", req.agent_id.c_str());
		res.success = true;
	} else {
		res.success = false;
	}

	return true;
}

// Each agent node publishes to the same topic, each publish updates the topic and received here one by one. 
// It should be tested if all of them are successfully caught
void ChargingManagement::receiveRobotHeartbeat(const auto_smart_factory::RobotHeartbeat &hb) {
	std::string idStr = hb.id;
	// ROS_INFO("[charging management] Tracked agents number is : %d", trackedRobots.size());
	// ROS_INFO("[charging management] Registered agents number is : %d", registeredRobots.size());
	if (registeredRobots.count(idStr) == 0){
		ROS_WARN("Receiving heartbeats from unregistered robots [%s]! Please register ASAP", idStr.c_str());
	}
	else{
		if(hb.battery_level == 0.0) ROS_WARN("%s DIED!!!!!!!! out of battery!", idStr.c_str());
	robotsPosition[idStr] = hb.position;
//	ROS_INFO("[charging management] RobotHeartbeat from  %s received and the battery level is %f !", idStr.c_str(), hb.battery_level);
	}

	if(hb.battery_level < ChargingManagement::CRITICAL){  //TODO: set warning battery level according to robots config
		if (registeredRobots[idStr].second == "untracked"){
			registeredRobots[idStr].second = "unassigned";
			trackedRobots.insert(std::pair<std::string,int>(idStr, -1));
			ROS_INFO("[charging management] Detect %s needs charging, starting tracking", idStr.c_str());
		}
	}

	if( registeredRobots[idStr].second == "requested" && hb.battery_level == ChargingManagement::CHARGED){
		registeredRobots[idStr].second = "charged";
	}	
}

bool ChargingManagement::assignRobot(std::string robotId, int tray_id, bool end) const {
	ros::NodeHandle n;
	ros::ServiceClient assignChargingTaskClient = n.serviceClient<auto_smart_factory::AssignChargingTask>(
			"/" + robotId + "/assign_charging_task");

	auto_smart_factory::AssignChargingTask srv;
	srv.request.tray_id = tray_id;
	srv.request.end = end;

	if (assignChargingTaskClient.call(srv)) {
	//	ROS_INFO("Successfully assign %s charging task", robotId.c_str());
		return srv.response.success;
	}
	return false;
}

int ChargingManagement::getRobotETA(geometry_msgs::Point robotPose, geometry_msgs::Point charging_position){
	ros::NodeHandle n;
	// TODO: Implement the ETA service under an ETA agent and call it here to calculate robot's estimated time to arrive to
	// a charging position. OPTIONALLY, you are encouraged to come with your own ideas to find the best station, no must to request for ETA
	// An example ROS service (note that it is not implemented!):
	// ros::ServiceClient getRobotETAclient = n.serviceClient<auto_smart_factory::CalculateETA>(
	//		"/eta/calculate_task_ETA");

	// auto_smart_factory::CalculateETA srv;
	// srv.request.posStart = robotPose;
	// srv.request.posEnd = charging_position;

	// if (getRobotETAclient.call(srv)) {
	// 	return srv.response.eta_value;
	//}
	return 0; // return dummy. will be updated
}

int ChargingManagement::findBestStation(std::string robotId){
	geometry_msgs::Point chargingStation;
	int distance;
	std::pair<int, int> minDistance;
	if(freeChargingStations.size() < 1)  return -1;
	ROS_INFO("There are %lu free charging stations", freeChargingStations.size());

	for(int i = 0; i < freeChargingStations.size(); i++){
		chargingStation.x = freeChargingStations[i].x;
		chargingStation.y = freeChargingStations[i].y;

		distance = getRobotETA(robotsPosition[robotId], chargingStation);

		if (i == 0) {
			minDistance.second = distance;
			minDistance.first = i;
		}
		else if(distance < minDistance.second){
			minDistance.second = distance;
			minDistance.first = i;
		}

	}
	return minDistance.first;
}

bool ChargingManagement::getFreeChargingStations(
		auto_smart_factory::GetFreeChargingStations::Request &req,
		auto_smart_factory::GetFreeChargingStations::Response &res) {
	res.charging_stations = freeChargingStations;
	return true;
}
