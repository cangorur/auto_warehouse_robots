#include "agent/Agent.h"

#include "warehouse_management/WarehouseManagement.h"

Agent::Agent(std::string agent_id) {
	agentID = agent_id;
	std::string idStr = agentID.substr(agentID.find('_') + 1);
	agentIdInt = std::stoi(idStr);
	position.z = -1;
	map = nullptr;

	ros::NodeHandle pn("~");
	//setup init_agent service
	pn.setParam(agentID, "~init");
	init_srv = pn.advertiseService("init", &Agent::init, this);
}

Agent::~Agent() {
	motionPlanner->~MotionPlanner();
	gripper->~Gripper();
	obstacleDetection->~ObstacleDetection();
	map->~Map();
	chargingManagement->~ChargingManagement();
	taskHandler->~TaskHandler();
	reservationManager->~ReservationManager();
}

void Agent::update() {
	if(isInitializedCompletely()) {
		// Register at task planner if not already done
		if(!registered && registerAgent()) {
			setupTaskHandling();
		}
		if(isTimeForHeartbeat()) {
			sendHeartbeat();
		}
		
		// Update Map and Reservations
		reservationManager->update();
				
		/* Task Execution */
		taskHandler->update();				
	}
}

bool Agent::init(auto_smart_factory::InitAgent::Request& req, auto_smart_factory::InitAgent::Response& res) {
	if(!initialized) {
		initialized = initialize(req.warehouse_configuration, req.robot_configuration);
		if(initialized) {
			ROS_INFO("initialize %s %.2f", agentID.c_str(), ros::Time::now().toSec());
			ROS_INFO("[%s]: Succesfully initialized!", agentID.c_str());
		} else
			ROS_ERROR("[%s]: Failed to initialize!", agentID.c_str());
	} else
		ROS_WARN("[%s]: Has already been initialized!", agentID.c_str());
	res.success = initialized;
	return true;
}

bool Agent::initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration, auto_smart_factory::RobotConfiguration robot_configuration) {
	ros::NodeHandle pn("~");
	warehouseConfig = warehouse_configuration;
	robotConfig = robot_configuration;
	
	double maxTurningSpeedInDegree = Math::toDeg(robot_configuration.max_angular_vel);
	hardwareProfile = new RobotHardwareProfile(robot_configuration.max_linear_vel, maxTurningSpeedInDegree, robot_configuration.discharging_rate, 0.f);
	//ROS_INFO("[%s]: MaxSpeed: %f m/s | MaxTurningSpeed: %f deg/s", agentID.c_str(), robot_configuration.max_linear_vel,maxTurningSpeedInDegree);

	pose_sub = n.subscribe(agentID + "/pose", 1, &Agent::poseCallback, this);
	battery_sub = n.subscribe(agentID + "/battery", 1, &Agent::batteryCallback, this);
	hokuyo_sub = n.subscribe(agentID + "/laser_scanner", 1, &Agent::laserCallback, this);
	motion_pub = pn.advertise<geometry_msgs::Twist>("motion", 1);
	heartbeat_pub = n.advertise<auto_smart_factory::RobotHeartbeat>("robot_heartbeats", 1);
	gripper_state_pub = pn.advertise<auto_smart_factory::GripperState>("gripper_state", 1);
	task_announce_sub = n.subscribe("/task_planner/task_broadcast", 1, &Agent::announcementCallback, this);
	taskrating_pub = pn.advertise<auto_smart_factory::TaskRating>("/task_response", 1);
	// TODO: Below topic can give some hints (example information an agent may need). They are not published in any of the nodes
	// collision_alert_sub = n.subscribe("/collisionAlert", 1, &Agent::collisionAlertCallback, this);

	visualisationPublisher = pn.advertise<visualization_msgs::Marker>("visualization_" + agentID, 100, true);
	vizPublicationTimer = pn.createTimer(ros::Duration(0.25f), &Agent::publishVisualisation, this); // in seconds
	
	reservationCoordination_pub = pn.advertise<auto_smart_factory::ReservationCoordination>("/reservation_coordination", 100, true);
	reservationCoordination_sub = pn.subscribe("/reservation_coordination", 100, &Agent::reservationCoordinationCallback, this);

	try {
		motionPlanner = new MotionPlanner(this, robotConfig, &(motion_pub));
		gripper = new Gripper(this, &(gripper_state_pub));

		// Disable to prevent crash because obstacleDetections was not initialized properly because no occupancy map is available
		obstacleDetection = new ObstacleDetection(agentID, *motionPlanner, robotConfig, warehouseConfig);
		obstacleDetection->enable(false);

		// Generate map
		std::vector<Rectangle> obstacles;
		for(auto o : warehouseConfig.map_configuration.obstacles) {
			obstacles.emplace_back(Point(o.posX, o.posY), Point(o.sizeX, o.sizeY), o.rotation);
		}
		map = new Map(warehouseConfig, obstacles, hardwareProfile, agentIdInt);

		// Charging Management
		chargingManagement = new ChargingManagement(this, warehouseConfig, map);

		// Reservation Manager
		reservationManager = new ReservationManager(&reservationCoordination_pub, map, agentIdInt, static_cast<int>(warehouse_configuration.robots.size()));
		
		// Task Handler
		taskHandler = new TaskHandler(this, &(taskrating_pub), map, motionPlanner, gripper, chargingManagement, reservationManager);
		
		// Agent color
		double color_r = 200;
		double color_g = 200;
		double color_b = 200;
		pn.getParam("color_r", color_r);
		pn.getParam("color_g", color_g);
		pn.getParam("color_b", color_b);
		agentColor.a = 1.0f;
		agentColor.r = static_cast<float>(color_r / 255.f);
		agentColor.g = static_cast<float>(color_g / 255.f);
		agentColor.b = static_cast<float>(color_b / 255.f);
						
		ROS_WARN("Finished Initialize [%s]", agentID.c_str());
		return true;
	} catch(...) {
		ROS_ERROR("[%s]: Exception occured!", agentID.c_str());
		return false;
	}
}

bool Agent::registerAgent() {
	std::string srv_name = "task_planner/register_agent";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::RegisterAgent>(srv_name);
	auto_smart_factory::RegisterAgent srv;
	srv.request.agent_id = agentID;
	srv.request.robot_configuration = robotConfig;
	ros::service::waitForService(srv_name);

	// for testing register just one agent
	//if (agentID.compare("robot_1") == 0 && client.call(srv)){
	if(client.call(srv)) {
		if(!registered) {
			registered = (bool) srv.response.success;
			if(registered)
				ROS_INFO("[%s]: Succesfully registered!", agentID.c_str());
			else
				ROS_ERROR("[%s]: Failed to register!", agentID.c_str());
		} else
			ROS_WARN("[%s]: Has already been registered!", agentID.c_str());
	} else {
		ROS_ERROR("[%s]: Failed to call service %s!", agentID.c_str(), srv_name.c_str());
	}
	return registered;
}

/*
 * Sets up services to communicate with task planner.
 */
void Agent::setupTaskHandling() {
	ros::NodeHandle pn("~");
	pn.setParam(agentID, "~assign_task");
	assign_task_srv = pn.advertiseService("assign_task", &Agent::assignTask, this);
}

bool Agent::isTimeForHeartbeat() {
	timeval time;
	gettimeofday(&time, 0);
	return lastHeartbeat == 0 || time.tv_sec - lastHeartbeat >= heartbeatPeriod;
}

void Agent::sendHeartbeat() {
	auto_smart_factory::RobotHeartbeat heartbeat;
	heartbeat.id = agentID;
	heartbeat.idle = taskHandler->isIdle();
	// TODO: An ETA or a buffer indicating the current workload (needs to be associated in a quantitative way)
	// Task planner will directly receive this to decide on which agent is the most available.
	// heartbeat.eta = Agent::getETA();
	if(position.z >= 0) {
		heartbeat.position = position;
		tf::Quaternion q;
		tf::quaternionMsgToTF(orientation, q);
		heartbeat.orientation = tf::getYaw(q);
	}
	heartbeat.battery_level = batteryLevel;
	heartbeat_pub.publish(heartbeat);
	updateTimer();
	ROS_DEBUG("[%s]: Heartbeat: idle=%s!", agentID.c_str(), taskHandler->isIdle() ? "true" : "false");
}

void Agent::updateTimer() {
	timeval time;
	gettimeofday(&time, 0);
	lastHeartbeat = time.tv_sec;
}

void Agent::collisionAlertCallback(const auto_smart_factory::CollisionAction& msg) {
	// TODO: this callback halts the motion when a collision detected. But currently there is no topic publishing such an alert
	// This function is left here as a hint. This alert can be sent by the obstacle detector, or by the central path/traffic planer.
	// Please examine CollisionAction.msg if you would like to use it.
	// It can be updated to take another strategies to avoid further collisions.
	if(msg.RobotId == agentID) {
		ROS_WARN("Got collision alert [%s] of length %.2f with halt %i", agentID.c_str(), msg.time_to_halt, msg.halt);
		obstacleDetection->enable(false);
		motionPlanner->stop();
		if(msg.halt) {
			ros::Duration(1 * msg.time_to_halt).sleep();
			motionPlanner->start();
			obstacleDetection->enable(true);
		}
	}
}

bool Agent::assignTask(auto_smart_factory::AssignTask::Request& req, auto_smart_factory::AssignTask::Response& res) {
	try {
		// ROS_INFO("[%s]: IN Agent::assignTask, number of tasks in queue: %i", agentID.c_str(), taskHandler->numberQueuedTasks());

		unsigned int task_id = req.task_id;
		auto_smart_factory::Tray input_tray = getTray(req.input_tray);
		auto_smart_factory::Tray storage_tray = getTray(req.storage_tray);

		// ROS_INFO("[%s]: AssignTask --> inputTray (x=%f, y=%f)", agentID.c_str(), input_tray.x, input_tray.y);
		// ROS_INFO("[%s]: AssignTask --> storageTray (x=%f, y=%f)", agentID.c_str(), storage_tray.x, storage_tray.y);

		// create Task and add it to task handler
		Path sourcePath;
		Path targetPath;
		bool success = false;

		double now = ros::Time::now().toSec();
		Task* lastTask = taskHandler->getLastTask();
		if(lastTask != nullptr){
			sourcePath = map->getThetaStarPath(lastTask->getTargetPosition(), input_tray, lastTask->getEndTime(), TransportationTask::getPickUpTime());
			
			if(sourcePath.isValid()) {
				targetPath = map->getThetaStarPath(input_tray, storage_tray, lastTask->getEndTime() + sourcePath.getDuration() + TransportationTask::getPickUpTime(), TransportationTask::getDropOffTime());
				
				if(targetPath.isValid()) {
					taskHandler->addTransportationTask(task_id, req.input_tray, req.storage_tray, sourcePath, targetPath, lastTask->getEndTime());
					success = true;
				}
			}
		} else {
			sourcePath = map->getThetaStarPath(getCurrentOrientedPosition(), input_tray, now, TransportationTask::getPickUpTime());
			
			if(sourcePath.isValid()) {
				targetPath = map->getThetaStarPath(input_tray, storage_tray, now + sourcePath.getDuration() + TransportationTask::getPickUpTime(), TransportationTask::getDropOffTime());
				
				if(targetPath.isValid()) {
					taskHandler->addTransportationTask(task_id, req.input_tray, req.storage_tray, sourcePath, targetPath, now);
					success = true;
				}
			}
		}

		res.success = success;
		if(!success) {
			ROS_WARN("[%s] task %d can not be assigned, as source or target path is invalid", agentID.c_str(), task_id);		
		}		
	} catch(std::out_of_range& e) {
		// task does not exist
		ROS_FATAL("[%d]: Attempted to assign inexistent task (specified id: %d)", agentIdInt, req.task_id);
		res.success = false;
	}
	
	return res.success;
}

auto_smart_factory::Tray Agent::getTray(unsigned int tray_id) {
	for(auto& tray : warehouseConfig.trays) {
		if(tray_id == tray.id) {
			return tray;
		}
	}
	ROS_FATAL("[%s]: Tray with id %u inexistent!", agentID.c_str(), tray_id);
	throw std::invalid_argument("Tray id was not found");
}

void Agent::poseCallback(const geometry_msgs::PoseStamped& msg) {
	position = msg.pose.position;
	orientation = msg.pose.orientation;

	// obstacleDetection->enable(true);
	tf::Quaternion q;
	tf::quaternionMsgToTF(orientation, q);
	motionPlanner->update(position, tf::getYaw(q));
}

void Agent::laserCallback(const sensor_msgs::LaserScan& msg) {
	ROS_DEBUG("[%s]: Laser callback: angle_min=%f, angle_max=%f, ranges_size=%lu", agentID.c_str(), msg.angle_min, msg.angle_max, msg.ranges.size());
	if(obstacleDetection->isEnabled()) {
		obstacleDetection->update(position, asin(orientation.z), msg);
	}
}

void Agent::batteryCallback(const std_msgs::Float32& msg) {
	batteryLevel = msg.data;
	ROS_DEBUG("[%s]: Battery Level: %f!", agentID.c_str(), batteryLevel);
}

void Agent::announcementCallback(const auto_smart_factory::TaskAnnouncement& taskAnnouncement) {
	taskHandler->announcementCallback(taskAnnouncement);
}

std::string Agent::getAgentID() {
	return agentID;
}

int Agent::getAgentIdInt() {
	return agentIdInt;
}

float Agent::getAgentBattery() {
	return batteryLevel;
}

geometry_msgs::Point Agent::getCurrentPosition() {
	return position;
}

geometry_msgs::Quaternion Agent::getCurrentOrientation() {
	return orientation;
}

OrientedPoint Agent::getCurrentOrientedPosition() const {
	tf::Quaternion q;
	tf::quaternionMsgToTF(orientation, q);
	
	return OrientedPoint(position.x, position.y, tf::getYaw(q));
}

void Agent::publishVisualisation(const ros::TimerEvent& e) {
	if(map != nullptr) {
		visualisationPublisher.publish(map->getObstacleVisualization());
		
		auto reservationMsg = map->getInactiveReservationVisualization(agentIdInt, getAgentColor());
		if(!reservationMsg.points.empty()) {
			visualisationPublisher.publish(reservationMsg);
		}

		reservationMsg = map->getActiveReservationVisualization(agentIdInt, getAgentColor());
		if(!reservationMsg.points.empty()) {
			visualisationPublisher.publish(reservationMsg);
		}
	}
}

ros::Publisher* Agent::getVisualisationPublisher() {
	return &visualisationPublisher;
}

void Agent::reservationCoordinationCallback(const auto_smart_factory::ReservationCoordination& msg) {
	reservationManager->reservationCoordinationCallback(msg);
}

bool Agent::isInitializedCompletely() {
	return initialized && motionPlanner->isPositionInitialized();
}

std_msgs::ColorRGBA Agent::getAgentColor() {
	return agentColor;
}
