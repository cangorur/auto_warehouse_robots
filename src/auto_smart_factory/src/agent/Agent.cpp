#include "agent/Agent.h"
#include "Math.h"

#include <tf/transform_datatypes.h>

Agent::Agent(std::string agent_id) {
	agentID = agent_id;
	agentIdInt = std::stoi(agentID.substr(6, 1));
	position.z = -1;
	map = nullptr;

	ros::NodeHandle pn("~");
	//setup init_agent service
	pn.setParam(agentID, "~init");
	init_srv = pn.advertiseService("init", &Agent::init, this);

	visualisationPublisher = pn.advertise<visualization_msgs::Marker>("visualization_" + agent_id, 100);
	vizPublicationTimer = pn.createTimer(ros::Duration(0.5f), &Agent::publishVisualisation, this); // in seconds
}

Agent::~Agent() {
	this->motionPlanner->~MotionPlanner();
	this->gripper->~Gripper();
	this->obstacleDetection->~ObstacleDetection();
	this->map->~Map();
	this->chargingManagement->~ChargingManagement();
	this->taskHandler->~TaskHandler();
}

void Agent::update() {
	if(initialized) {
		// Register at taskplanner if not already done
		if(!registered && registerAgent()) {
			setupTaskHandling();
		}
		if(isTimeForHeartbeat()) {
			sendHeartbeat();
		}
		
		// Update Map
		map->deleteExpiredReservations(ros::Time::now().toSec());
		
		/* Task Execution */
		this->taskHandler->update();		

		//PathPlanning
		/*if(!isPathSet) {
			if(getCurrentPosition().x != 0 && map != nullptr) {
				Path p = map->getThetaStarPath(Point(this->getCurrentPosition()), Point(1, agentIdInt + 1), 0);

				this->motionPlanner->newPath(p);
				if(p.getDistance() > 0) {
					this->motionPlanner->enable(true);
					this->motionPlanner->start();
					isPathSet = true;
				}
			}
		}*/
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
	
	float maxTurningSpeedInDegree = Math::toDeg(robot_configuration.max_angular_vel);
	hardwareProfile = new RobotHardwareProfile(robot_configuration.max_linear_vel, maxTurningSpeedInDegree, robot_configuration.discharging_rate, 0.f);
	ROS_INFO("[%s]: MaxSpeed: %f m/s | MaxTurningSpeed: %f deg/s", agentID.c_str(), robot_configuration.max_linear_vel,maxTurningSpeedInDegree);

	if(!setupIdlePosition()) {
		return false;
	}
	this->pose_sub = n.subscribe(agentID + "/pose", 1, &Agent::poseCallback, this);
	this->battery_sub = n.subscribe(agentID + "/battery", 1, &Agent::batteryCallback, this);
	this->hokuyo_sub = n.subscribe(agentID + "/laser_scanner", 1, &Agent::laserCallback, this);
	this->motion_pub = pn.advertise<geometry_msgs::Twist>("motion", 1);
	this->heartbeat_pub = n.advertise<auto_smart_factory::RobotHeartbeat>("robot_heartbeats", 1);
	this->gripper_state_pub = pn.advertise<auto_smart_factory::GripperState>("gripper_state", 1);
	this->additional_time_pub = pn.advertise<auto_smart_factory::AdditionalTime>("additional_time", 1);
	this->task_announce_sub = n.subscribe("/task_planner/task_broadcast", 1, &Agent::announcementCallback, this);
	this->taskrating_pub = pn.advertise<auto_smart_factory::TaskRating>("/task_response", 1);
	// TODO: Below topic can give some hints (example information an agent may need). They are not published in any of the nodes
	// this->collision_alert_sub = n.subscribe("/collisionAlert", 1, &Agent::collisionAlertCallback, this);

	try {
		this->motionPlanner = new MotionPlanner(this, this->robotConfig, &(this->motion_pub));
		this->gripper = new Gripper(this, &(this->gripper_state_pub));

		// Disable to prevent crash because obstacleDetections was not initialized properly because no occupancy map is available
		this->obstacleDetection = new ObstacleDetection(agentID, *motionPlanner, robotConfig, warehouseConfig);
		this->obstacleDetection->enable(false);

		// Generate map
		std::vector<Rectangle> obstacles;
		for(auto o : warehouseConfig.map_configuration.obstacles) {
			obstacles.emplace_back(Point(o.posX, o.posY), Point(o.sizeX, o.sizeY), o.rotation);
		}
		this->map = new Map(warehouseConfig, obstacles, hardwareProfile);

		// Charging MAnagement
		this->chargingManagement = new ChargingManagement(this);

		// Task Handler
		this->taskHandler = new TaskHandler(agentID, &(this->taskrating_pub), this->map, this->motionPlanner, this->gripper, 
			this->chargingManagement);

		ROS_WARN("Finished Initialize [%s]", agentID.c_str());
		return true;
	} catch(...) {
		ROS_ERROR("[%s]: Exception occured!", agentID.c_str());
		return false;
	}
}

bool Agent::setupIdlePosition() {
	for(auto& idle_position : warehouseConfig.idle_positions)
		if(idle_position.id == agentID) {

			idlePosition.x = idle_position.pose.x;
			idlePosition.y = idle_position.pose.y;
			double rad = idle_position.pose.theta / 180.0 * PI;
			idleOrientationPoint.x = idlePosition.x + cos(rad) * 1.0;
			idleOrientationPoint.y = idlePosition.y + sin(rad) * 1.0;
			return true;
		}
	ROS_ERROR("[%s]: Failed to setup idle position!", agentID.c_str());
	return false;
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
	this->assign_task_srv = pn.advertiseService("assign_task", &Agent::assignTask, this);
}

void Agent::setIdle(bool idle) {
	if((isIdle && !idle) || (!isIdle && idle)) {
		isIdle = idle;
		sendHeartbeat();
	}
}

bool Agent::isTimeForHeartbeat() {
	timeval time;
	gettimeofday(&time, 0);
	return lastTimestamp == 0 || time.tv_sec - lastTimestamp >= breakDuration;
}

void Agent::sendHeartbeat() {
	auto_smart_factory::RobotHeartbeat heartbeat;
	heartbeat.id = agentID;
	heartbeat.idle = isIdle;
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
	this->heartbeat_pub.publish(heartbeat);
	updateTimer();
	ROS_DEBUG("[%s]: Heartbeat: idle=%s!", agentID.c_str(), isIdle ? "true" : "false");
}

void Agent::updateTimer() {
	timeval time;
	gettimeofday(&time, 0);
	lastTimestamp = time.tv_sec;
}

void Agent::collisionAlertCallback(const auto_smart_factory::CollisionAction& msg) {
	// TODO: this callback halts the motion when a collision detected. But currently there is no topic publishing such an alert
	// This function is left here as a hint. This alert can be sent by the obstacle detector, or by the central path/traffic planer.
	// Please examine CollisionAction.msg if you would like to use it.
	// It can be updated to take another strategies to avoid further collisions.
	if(msg.RobotId == agentID) {
		ROS_WARN("Got collision alert [%s] of length %.2f with halt %i", agentID.c_str(), msg.time_to_halt, msg.halt);
		this->obstacleDetection->enable(false);
		this->motionPlanner->stop();
		if(msg.halt) {
			ros::Duration(1 * msg.time_to_halt).sleep();
			this->motionPlanner->start();
			this->obstacleDetection->enable(true);
		}
	}
}

bool Agent::assignTask(auto_smart_factory::AssignTask::Request& req,
                       auto_smart_factory::AssignTask::Response& res) {
	try {
		if(isIdle) {
			ROS_INFO("[%s]: IN Agent::assignTask, number of tasks in queue: %i", agentID.c_str(), taskHandler->numberQueuedTasks());

			int task_id = req.task_id;
			auto_smart_factory::Tray input_tray = getTray(req.input_tray);
			auto_smart_factory::Tray storage_tray = getTray(req.storage_tray);

			ROS_INFO("[%s]: AssignTask --> inputTray (x=%f, y=%f)", agentID.c_str(), input_tray.x, input_tray.y);
			ROS_INFO("[%s]: AssignTask --> storageTray (x=%f, y=%f)", agentID.c_str(), storage_tray.x, storage_tray.y);

			// create Task and add it to task handler
			OrientedPoint sourcePos = map->getPointInFrontOfTray(input_tray);
			OrientedPoint targetPos = map->getPointInFrontOfTray(storage_tray);
			Path sourcePath;
			Path targetPath;

			Task* lastTask = taskHandler->getLastTask();
			if(lastTask != nullptr){
				sourcePath = map->getThetaStarPath(Point(lastTask->getTargetPosition()), input_tray, lastTask->getEndTime());
				targetPath = map->getThetaStarPath(input_tray, storage_tray, lastTask->getEndTime() + sourcePath.getDuration());
				taskHandler->addTransportationTask(task_id, req.input_tray, sourcePos, req.storage_tray, targetPos, sourcePath, 
					targetPath, lastTask->getEndTime());
			} else {
				// take the current position
				// TODO add correct time now
				double now = ros::Time::now().toSec();
				sourcePath = map->getThetaStarPath(Point(this->getCurrentPosition()), input_tray, now);
				targetPath = map->getThetaStarPath(input_tray, storage_tray, now + sourcePath.getDuration());
				taskHandler->addTransportationTask(task_id, req.input_tray, sourcePos, req.storage_tray, targetPos, sourcePath, 
					targetPath, now);
			}
			

			initialTimeOfCurrentTask = ros::Time::now().toSec();
			ROS_INFO("[%d]: Task %i successfully assigned at %.2f! Queue size is %i", agentIdInt, req.task_id, 
				ros::Time::now().toSec(), taskHandler->numberQueuedTasks());
			res.success = true;
		} else {
			ROS_WARN("[%d]: Is busy! - Task %i has not been assigned!", agentIdInt, req.task_id);
			res.success = false;
		}
	} catch(std::out_of_range& e) {
		// task does not exist
		ROS_ERROR("[%d]: Attempted to assign inexistent task (specified id: %d)", agentIdInt, req.task_id);
		res.success = false;
	}
	return res.success;
}

auto_smart_factory::Tray Agent::getTray(unsigned int tray_id) {
	for(auto& tray : warehouseConfig.trays)
		if(tray_id == tray.id) {
			return tray;
		}
	ROS_ERROR("[%s]: Tray with id %u inexistent!", agentID.c_str(), tray_id);
}


void Agent::poseCallback(const geometry_msgs::PoseStamped& msg) {
	position = msg.pose.position;
	orientation = msg.pose.orientation;

	// this->obstacleDetection->enable(true);
	tf::Quaternion q;
	tf::quaternionMsgToTF(orientation, q);
	this->motionPlanner->update(position, tf::getYaw(q));
}

void Agent::laserCallback(const sensor_msgs::LaserScan& msg) {
	ROS_DEBUG("[%s]: Laser callback: angle_min=%f, angle_max=%f, ranges_size=%lu",
	          agentID.c_str(), msg.angle_min, msg.angle_max, msg.ranges.size());
	if(this->obstacleDetection->isEnabled()) {
		this->obstacleDetection->update(position, asin(orientation.z), msg);
	}
}

void Agent::batteryCallback(const std_msgs::Float32& msg) {
	batteryLevel = msg.data;
	ROS_DEBUG("[%s]: Battery Level: %f!", agentID.c_str(), batteryLevel);
}

void Agent::announcementCallback(const auto_smart_factory::TaskAnnouncement& taskAnnouncement) {
	// TODO: compute this for all 
	std::vector< TrayScore* > scores;
	double queuedDuration = taskHandler->getDuration();
	double queuedBatteryConsumption = taskHandler->getBatteryConsumption();
	for(uint32_t it_id : taskAnnouncement.start_ids){
		for(uint32_t st_id : taskAnnouncement.end_ids){
			// get Path
			auto_smart_factory::Tray input_tray = getTray(it_id);
			auto_smart_factory::Tray storage_tray = getTray(st_id);
			Path sourcePath = Path();
			if(taskHandler->getLastTask() != nullptr){
				// take the last position of the last task
				sourcePath = map->getThetaStarPath(Point(taskHandler->getLastTask()->getTargetPosition()), input_tray, 0);
			} else {
				// take the current position
				sourcePath = map->getThetaStarPath(Point(this->getCurrentPosition()), input_tray, 0);
			}
			Path targetPath = map->getThetaStarPath(input_tray, storage_tray, 0);
			// get Battery consumption
			double batCons = queuedBatteryConsumption + sourcePath.getBatteryConsumption() + targetPath.getBatteryConsumption();
			// check battery consumption
			if(chargingManagement->isEnergyAvailable(batCons)){
				// get Path duration pickup and dropoff constants are ignored as every robot has to do them
				double duration = queuedDuration + sourcePath.getDuration() + targetPath.getDuration();
				// get score multiplier
				double scoreFactor = chargingManagement->getScoreMultiplier(batCons);
				// compute score
				double score = (1/scoreFactor) * duration;
				// add score to list
				scores.push_back(new TrayScore(it_id, st_id, score));
			}
		}
	}
	if(!scores.empty()){
		std::sort(scores.begin(), scores.end(),
			[](TrayScore* first, TrayScore* second) 
				{return first->score < second->score;}
			);
		TrayScore* best = scores.front();
		// publish score
		this->taskHandler->publishScore(taskAnnouncement.request_id, best->score, best->sourceTray, best->targetTray);
		// clean list
		for(TrayScore* s : scores){
			delete s;
		}
		scores.clear();
	} else {
		// reject task
		this->taskHandler->rejectTask(taskAnnouncement.request_id);
	}
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

void Agent::publishVisualisation(const ros::TimerEvent& e) {
	if(map != nullptr) {
		visualisationPublisher.publish(map->getObstacleVisualization());
		
		auto reservationMsg = map->getReservationVisualization();
		if(!reservationMsg.points.empty()) {
			visualisationPublisher.publish(reservationMsg);
		}
	}
}

ros::Publisher* Agent::getVisualisationPublisher() {
	return &visualisationPublisher;
}

