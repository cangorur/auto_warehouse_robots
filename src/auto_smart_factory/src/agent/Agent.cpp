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
	vizPublicationTimer = pn.createTimer(ros::Duration(1), &Agent::publishVisualization, this);
}

Agent::~Agent() {
	this->motionPlanner->~MotionPlanner();
	this->gripper->~Gripper();
	this->obstacleDetection->~ObstacleDetection();
	this->map->~Map();
}

void Agent::update() {
	if(initialized) {
		//register at taskplanner if not already done
		if(!registered && registerAgent()) {
			setupTaskHandling();
		}

		//send heartbeat if the time has come to do so
		if(isTimeForHeartbeat()) {
			sendHeartbeat();
		}

		/* PathPlanning
		setState(true);

		if(!isPathSet) {
			if(getCurrentPosition().x != 0 && map != nullptr) {
				Path p = map->getThetaStarPath(Point(this->getCurrentPosition()), Point(1, agentIdInt + 1));

				this->motionPlanner->newPath(p);
				if(p.getLength() > 0) {
					this->motionPlanner->enable(true);
					this->motionPlanner->start();
					isPathSet = true;
				}
			}
		}
		*/

		// DEMO
		if (!isPathSet)
		{
			if (getCurrentPosition().x != 0)
			{
				// code for dummy path following
				//ROS_ERROR("Position: %.2f, %.2f", getCurrentPosition().x, getCurrentPosition().y);
				std::vector<geometry_msgs::Point> path;
				geometry_msgs::Point p1;
				p1.x = getCurrentPosition().x;
				p1.y = getCurrentPosition().y;
				path.push_back(p1);

				p1.x = 14.0;
				p1.y = 5.0;
				path.push_back(p1);
				p1.x = 12.5;
				p1.y = 2.0;
				path.push_back(p1);

				p1.x = 1.9;
				p1.y = 1.0;
				path.push_back(p1);
				p1.x = 1.5;
				p1.y = 1.11;
				path.push_back(p1);

				p1.x = 1.25;
				p1.y = 1.25;
				path.push_back(p1);

				p1.x = 1.11;
				p1.y = 1.5;
				path.push_back(p1);
				p1.x = 1.0;
				p1.y = 1.9;
				path.push_back(p1);

				p1.x = 1.0;
				p1.y = 13.0;
				path.push_back(p1);

				// End point
				geometry_msgs::Point p5;
				p5.x = 12.5;
				p5.y = 14.0;

				if (agentID == "robot_2")
				{
					ros::Duration(10).sleep();
					p5.y = 13.5;
				}
				if (agentID == "robot_3")
				{
					ros::Duration(20).sleep();
					p5.y = 13.0;
				}
				if (agentID == "robot_4")
				{
					ros::Duration(30).sleep();
					p5.y = 12.5;
				}
				if (agentID == "robot_5")
				{
					ros::Duration(40).sleep();
					p5.y = 12.0;
				}
				if (agentID == "robot_6")
				{
					ros::Duration(50).sleep();
					p5.y = 11.5;
				}
				if (agentID == "robot_7")
				{
					ros::Duration(60).sleep();
					p5.y = 11.0;
				}
				if (agentID == "robot_8")
				{
					ros::Duration(70).sleep();
					p5.y = 10.5;
				}

				path.push_back(p5);

				this->motionPlanner->newPath(this->getCurrentPosition(), path, p5, false);
				this->motionPlanner->enable(true);
				this->motionPlanner->start();

				isPathSet = true;
			}
		}
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
		
		this->map = new Map(warehouseConfig.map_configuration.width, warehouseConfig.map_configuration.height, warehouseConfig.map_configuration.margin, warehouseConfig.map_configuration.resolutionThetaStar, obstacles);
		
		return true;
	} catch(...) {
		ROS_ERROR("[%s]: Exception occured!", agentID.c_str());
		return false;
	}
	ROS_WARN("Finished Initialize [%s]", agentID.c_str());
}

bool Agent::setupIdlePosition() {
	for(int i = 0; i < warehouseConfig.idle_positions.size(); i++)
		if(warehouseConfig.idle_positions[i].id == agentID) {

			idlePosition.x = warehouseConfig.idle_positions[i].pose.x;
			idlePosition.y = warehouseConfig.idle_positions[i].pose.y;
			double rad = warehouseConfig.idle_positions[i].pose.theta / 180.0 * PI;
			idleOrientationPoint.x = idlePosition.x + cos(rad) * 1.0;
			idleOrientationPoint.y = idlePosition.y + sin(rad) * 1.0;
			return true;
		}
	ROS_ERROR("[%s]: Failed to setup idle position!", agentID.c_str());
	return false;
}

bool Agent::registerAgent() {
	std::string srv_name = "task_planner/register_agent";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::RegisterAgent>(srv_name.c_str());
	auto_smart_factory::RegisterAgent srv;
	srv.request.agent_id = agentID;
	srv.request.robot_configuration = robotConfig;
	ros::service::waitForService(srv_name.c_str());

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

void Agent::setState(bool idle) {
	if((isIdle && !idle) || (!isIdle && idle)) {
		isIdle = idle;
		sendHeartbeat();
	}
}

bool Agent::isTimeForHeartbeat() {
	timeval time;
	gettimeofday(&time, 0);
	if(lastTimestamp == 0 || time.tv_sec - lastTimestamp >= breakDuration) {
		return true;
	}
	return false;
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
			ROS_INFO("[%s]: IN Agent::assignTask", agentID.c_str());

			int task_id = req.task_id;
			auto_smart_factory::Tray input_tray = getTray(req.input_tray);
			auto_smart_factory::Tray storage_tray = getTray(req.storage_tray);

			geometry_msgs::Point input_tray_position;
			geometry_msgs::Point storage_tray_position;

			input_tray_position.x = input_tray.x;
			input_tray_position.y = input_tray.y;
			storage_tray_position.x = storage_tray.x;
			storage_tray_position.y = storage_tray.y;

			ROS_INFO("[%s]: AssignTask --> inputTray (x=%f, y=%f)", agentID.c_str(), input_tray.x, input_tray.y);
			ROS_INFO("[%s]: AssignTask --> storageTray (x=%f, y=%f)", agentID.c_str(), storage_tray.x, storage_tray.y);

			geometry_msgs::Point input_drive_point;
			geometry_msgs::Point storage_drive_point;
			geometry_msgs::Point input_drive_back_point;
			geometry_msgs::Point storage_drive_back_point;
			geometry_msgs::Point input_approach_point;
			geometry_msgs::Point storage_approach_point;

			// don't drive to the tray exactly, stop a bit in front for (un)loading
			double input_dx = cos(input_tray.orientation * PI / 180);
			double input_dy = sin(input_tray.orientation * PI / 180);
			double storage_dx = cos(storage_tray.orientation * PI / 180);
			double storage_dy = sin(storage_tray.orientation * PI / 180);
			input_drive_point.x = input_tray_position.x + 0.5 * input_dx;
			input_drive_point.y = input_tray_position.y + 0.5 * input_dy;
			input_drive_back_point.x = input_tray_position.x + 1.3 * input_dx;
			input_drive_back_point.y = input_tray_position.y + 1.3 * input_dy;
			input_approach_point.x = input_tray_position.x + input_dx;
			input_approach_point.y = input_tray_position.y + input_dy;
			storage_drive_point.x = storage_tray_position.x + 0.5 * storage_dx;
			storage_drive_point.y = storage_tray_position.y + 0.5 * storage_dy;
			storage_drive_back_point.x = storage_tray_position.x + 1.3 * storage_dx;
			storage_drive_back_point.y = storage_tray_position.y + 1.3 * storage_dy;
			storage_approach_point.x = storage_tray_position.x + 1.5 * storage_dx;
			storage_approach_point.y = storage_tray_position.y + 1.5 * storage_dy;


			//Path path_to_input_tray(agentID, task_id, GOAL::LOAD, position, input_drive_point, input_approach_point, input_tray_position, input_drive_back_point, true);

			//    When Robot arrived at input_tray it will start traveling from input tray to output tray
			//Path path_to_storage_tray(agentID, task_id, GOAL::UNLOAD, input_drive_back_point,
			//            storage_drive_point, storage_approach_point, storage_tray_position, storage_drive_back_point, false);

			// 2- Set currentPath
			//setCurrentPath(path_to_input_tray);
			//hasDriven = true;

			// 3- Add the remaining paths to the pathsStack
			//pathsStack.push(path_to_storage_tray);

			ROS_INFO("[%s]: Task %i successfully assigned!", agentID.c_str(), req.task_id);
			initialTimeOfCurrentTask = ros::Time::now().toSec();
			ROS_INFO("assignTask %s %.2f %i", agentID.c_str(), initialTimeOfCurrentTask, task_id);
			setState(false);     //Set to non idle if a task is assigned
			res.success = true;
		} else {
			ROS_WARN("[%s]: Is busy! - Task %i has not been assigned!",
			         agentID.c_str(), req.task_id);
			res.success = false;
		}
	} catch(std::out_of_range& e) {
		// task does not exist
		ROS_ERROR("[%s]: Attempted to assign inexistent task (specified id: %d)",
		          agentID.c_str(), req.task_id);
		res.success = false;
	}
	return res.success;
}

auto_smart_factory::Tray Agent::getTray(unsigned int tray_id) {
	for(int i = 0; i < warehouseConfig.trays.size(); i++)
		if(tray_id == warehouseConfig.trays[i].id) {
			return warehouseConfig.trays[i];
		}
	ROS_ERROR("[%s]: Tray with id %u inexistent!",
	          agentID.c_str(), tray_id);
}


void Agent::poseCallback(const geometry_msgs::PoseStamped& msg) {
	position = msg.pose.position;
	orientation = msg.pose.orientation;

	if(this->motionPlanner->isEnabled()) {
		// this->obstacleDetection->enable(true);
		tf::Quaternion q;
		tf::quaternionMsgToTF(orientation, q);
		this->motionPlanner->update(position, tf::getYaw(q));
	} else {
		// this->obstacleDetection->enable(false);
	}
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

std::string Agent::getAgentID() {
	return agentID;
}

geometry_msgs::Point Agent::getCurrentPosition() {
	return position;
}

geometry_msgs::Quaternion Agent::getCurrentOrientation() {
	return orientation;
}

void Agent::publishVisualization(const ros::TimerEvent& e) {
	if(map != nullptr) {
		visualisationPublisher.publish(map->getVisualization());
	}
}

ros::Publisher* Agent::getVisualisationPublisher() {
	return &visualisationPublisher;
}

