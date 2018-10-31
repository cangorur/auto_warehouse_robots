#include <agent/Agent.h>

Agent::Agent(std::string agent_id){
	agentID = agent_id;
	position.z = -1;

	ros::NodeHandle pn("~");
	//setup init_agent service
	pn.setParam(agentID, "~init");
	this->init_srv = pn.advertiseService("init", &Agent::init, this);
}

Agent::~Agent(){
	this->motionPlanner->~MotionPlanner();
	this->gripper->~Gripper();
	this->obstacleDetection->~ObstacleDetection();
}

void Agent::update(){
	if(initialized){

		//register at taskplanner if not already done
		if(!registered && registerAgent())
			setupTaskHandling();

		//register at charging stations if not already done
		 if(!registeredCharging && registerAgentCharging())
		 		setupChargingHandling();

		//send heartbeat if the time has come to do so
		if(isTimeForHeartbeat())
			sendHeartbeat();

		//handle idle state.
		//currentPlan variable holds the existing plan. It may be used for ETA calculation. Once it is executed robot directly navigates there
		int pathLength = static_cast<int>(currentPlan.getLength());
		if(currentPlan.isDone() && pathLength != 0){
		    if(currentPath.isDone()) {
		        if(pathsStack.empty()) {
		            this->motionPlanner->enable(false);
		            if(initialTimeOfCurrentTask > 0.0) {
                        double currentTime = ros::Time::now().toSec();
                        ROS_INFO("finalTimeOfTask %s %i %.2f", agentID.c_str(), currentPath.getTaskId(), currentTime);
                        ROS_INFO("durationOfTask %s %i %.2f", agentID.c_str(), currentPath.getTaskId(), currentTime - initialTimeOfCurrentTask);
                        initialTimeOfCurrentTask = 0.0;
		            }
					setState(true);    // Set to idle state if no paths
                    }
		        else {
		            setCurrentPath(pathsStack.top());
		            pathsStack.pop();
					ROS_WARN("[%s]Start new path, size: %lu",agentID.c_str(), pathsStack.size());
					setState(false);    // Remain not idle if has path
		            getTheNextPath();
		        }
		    }
		    else {
		        getTheNextPath();
		    }
		}
		//handle current_plan
		if(!currentPlan.isDone()) {
			currentPlan.execute(position, batteryLevel);
		}
	}
}

bool Agent::init(auto_smart_factory::InitAgent::Request  &req,
         auto_smart_factory::InitAgent::Response &res) {
	if(!initialized){
		initialized = initialize(req.warehouse_configuration, req.robot_configuration);
		if(initialized){
		    ROS_INFO("initialize %s %.2f", agentID.c_str(), ros::Time::now().toSec());
	  		ROS_INFO("[%s]: Succesfully initialized!", agentID.c_str());
	  		}
		else
	  		ROS_ERROR("[%s]: Failed to initialize!", agentID.c_str());
	} else
  		ROS_WARN("[%s]: Has already been initialized!", agentID.c_str());
	res.success = initialized;
  	return true;
}

bool Agent::initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration,
		auto_smart_factory::RobotConfiguration robot_configuration){
	ros::NodeHandle pn("~");
	warehouseConfig = warehouse_configuration;
	robotConfig = robot_configuration;

	if(!setupIdlePosition())
		return false;
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
		this->obstacleDetection = new ObstacleDetection(agentID, *motionPlanner,
			       robotConfig, warehouseConfig);
		return true;
	} catch (...) {
		ROS_ERROR("[%s]: Exception occured!", agentID.c_str());
		return false;
	}
	ROS_WARN("Finished Initialize [%s]", agentID.c_str());
}

bool Agent::setupIdlePosition() {
	for(int i=0; i < warehouseConfig.idle_positions.size(); i++)
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

bool Agent::registerAgent(){
	std::string srv_name = "task_planner/register_agent";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::RegisterAgent>(srv_name.c_str());
	auto_smart_factory::RegisterAgent srv;
	srv.request.agent_id = agentID;
	srv.request.robot_configuration = robotConfig;
	ros::service::waitForService(srv_name.c_str());

        // for testing register just one agent
	//if (agentID.compare("robot_1") == 0 && client.call(srv)){
	if (client.call(srv)) {
			if(!registered){
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

bool Agent::registerAgentCharging(){
	std::string srv_name = "charging_management/register_agent_charging_management";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::RegisterAgentCharging>(srv_name.c_str());
	auto_smart_factory::RegisterAgentCharging srv;
	srv.request.agent_id = agentID;
	srv.request.robot_configuration = robotConfig;
	srv.request.battery_level = batteryLevel;
	ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		if(!registeredCharging){
			registeredCharging = (bool) srv.response.success;
			if(registeredCharging)
				ROS_INFO("[%s]: Succesfully registered at charging station!", agentID.c_str());
			else
				ROS_ERROR("[%s]: Failed to register at charging station!", agentID.c_str());
		} else
  			ROS_WARN("[%s]: Has already been registered at charging station!", agentID.c_str());
	} else {
		ROS_ERROR("[%s]: Failed to call service %s!", agentID.c_str(), srv_name.c_str());
	}
	return registeredCharging;
}

/*
 * Sets up services to communicate with task planner.
 */
void Agent::setupTaskHandling(){
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

// Below ETA was a previous example. Needs to be updated with more sophisticated solutions
/* float Agent::getETA(){
    float eta_value;
    try {
        if (currentPlan.isDone()) {
            eta_value = 0;

        } else {
            eta_value = currentPlan.getLength();
        }
    } catch (...) {
        eta_value = -1;
    }
    return  eta_value;
}
*/

/*
 * Sets up services to communicate with charging station.
 */
void Agent::setupChargingHandling(){
	ros::NodeHandle pn("~");
	pn.setParam(agentID, "~assign_charging_task");  //?
	this->assign_charging_task_srv = pn.advertiseService("assign_charging_task", &Agent::assignChargingTask, this);
}

bool Agent::assignChargingTask(auto_smart_factory::AssignChargingTask::Request  &req,
			 					auto_smart_factory::AssignChargingTask::Response &res){

	auto_smart_factory::Tray charging_tray = getTray(req.tray_id);

	if(req.end){  //End charging task
		if(currentPath.getPathGoal() == GOAL::CHARGE){
			currentPlan.setDone(true);
			setState(true); //Set to idle state?
			res.success = true;
			ROS_INFO("[%s]Successfully end charging task", agentID.c_str());
			return true;
		}
		else {
			ROS_INFO("[%s]: Charging management trying to end a non exist charging plan!", agentID.c_str());
			res.success = false;
			return false;
		}
	}
	ROS_INFO("[%s]: Handling charging task", agentID.c_str());
	// This is the end position of this (charging) task
	// TODO: UPDATE THE CODE BELOW. The end position needs to be adjusted in a way that the robot always approaches to the tray facing front
	// Imagine that the charging contact points are on the front side of the robot.
	geometry_msgs::Point charging_tray_position, charging_dir_position;
	double charging_dx = cos(charging_tray.orientation * PI / 180);
    double charging_dy = sin(charging_tray.orientation * PI / 180);
    charging_tray_position.x = charging_tray.x + 0.5 * charging_dx;
    charging_tray_position.y = charging_tray.y + 0.5 * charging_dy;

	charging_dir_position.x = charging_tray.x + 1.5 * charging_dx;  //Face to opposite of charging tray in case of carrying
    charging_dir_position.y = charging_tray.y + 1.5 * charging_dy;

	//Check if the Agent is performing a task
	if (!isIdle) {
		ROS_INFO("[%s] getting a charging task while having task", agentID.c_str());

        // Check if the current Path is a load/unload or already a charging task
        // TODO: UPDATE BELOW The agent needs to decide what to do when a charging is needed while on a load / unload task
    	// ...
	    switch(currentPath.getPathGoal()) {
	        case GOAL::LOAD:{
				ROS_INFO("[%s] Need to go charge with the LOAD", agentID.c_str());
				Path path_to_charge(agentID, chargeTaskID, GOAL::CHARGE, position,
								charging_tray_position, charging_tray_position, charging_dir_position, charging_tray_position, false);

				 //Back start from charging position
				Path path_from_charge(agentID, currentPath.getTaskId(), GOAL::LOAD, charging_tray_position,
								currentPath.getEndPosition(), currentPath.getApproachPoint(), currentPath.getDirectionPoint(), currentPath.getDriveBackPoint(), false);
				pathsStack.push(path_from_charge);
				setCurrentPath(path_to_charge);
			}
	            break;
	        case GOAL::UNLOAD:{
				Path path_to_charge(agentID, chargeTaskID, GOAL::CHARGE, currentPath.getDriveBackPoint(),
								charging_tray_position, charging_tray_position, charging_dir_position, charging_tray_position, false);
				pathsStack.push(path_to_charge);
			}
	            break;
	        case GOAL::CHARGE:
				ROS_WARN("Got another charging assignment when charging!");
				break;
	        case GOAL::IDLE:{
				currentPlan.setDone(true);
				Path charging_path(agentID, chargeTaskID, GOAL::CHARGE, position, charging_tray_position, charging_tray_position, charging_dir_position, charging_tray_position, false);
    			setCurrentPath(charging_path);

				if(pathsStack.top().getPathGoal() == GOAL::CHARGE)
				ROS_INFO("[%s] IDLE + CHARGE -> next path goal: CHARGE ",agentID.c_str());
				else ROS_WARN("WRONG at IDLE + charge");
			}
				break;
	        default: ROS_INFO("Other cases exists!!! %s for charging",agentID.c_str());
	            break;
	    }
	}
	else {
			Path charging_path(agentID, chargeTaskID, GOAL::CHARGE, position, charging_tray_position, charging_tray_position, charging_dir_position, charging_tray_position, true);
    		setCurrentPath(charging_path);
	}

    //Create and set new current_path
    ROS_INFO("[%s]: Generating new path to charging station ID %d (x: %f, y: %f)", agentID.c_str(),req.tray_id, charging_tray_position.x, charging_tray_position.y);

    //Generate first chunk
    ROS_INFO("[%s]: Setting the first path chunk to charging station.", agentID.c_str());
    chargingStationId = req.tray_id;
    generateChargingPlan = true;
	setState(false);
	res.success = getTheNextPath();
	ROS_INFO("[%s]: Task charging assigned! success: %d", agentID.c_str(), res.success);
	return true;
}


void Agent::collisionAlertCallback(const auto_smart_factory::CollisionAction& msg) {
	// TODO: this callback halts the motion when a collision detected. But currently there is no topic publishing such an alert
	// This function is left here as a hint. This alert can be sent by the obstacle detector, or by the central path/traffic planer.
	// Please examine CollisionAction.msg if you would like to use it.
	// It can be updated to take another strategies to avoid further collisions.
	if (msg.RobotId == agentID) {
        ROS_WARN("Got collision alert [%s] of length %.2f with halt %i", agentID.c_str(), msg.time_to_halt, msg.halt);
        this->obstacleDetection->enable(false);
        this->motionPlanner->stop();
        if (msg.halt) {
            ros::Duration(1*msg.time_to_halt).sleep();
            this->motionPlanner->start();
            this->obstacleDetection->enable(true);
        }

    }
}

bool Agent::getTheNextPath() {
    // Requests a path from the Path_Planner
    // TODO: Based on the path planner design, update the request_new_path service response and request variables.
    std::string srv_name = "path_planner/request_new_path";
    ros::ServiceClient client = n.serviceClient<auto_smart_factory::RequestNewPath>(srv_name.c_str());
    auto_smart_factory::RequestNewPath srv;

    geometry_msgs::Point start_position = position;
	geometry_msgs::Point end_position = currentPath.getApproachPoint();      //request from path planner only until approach point
	// currentPath is an instance of Path class. Holds the paths / chunks assigned for the robot (see Path.cpp for more)
	bool next_chunk = !currentPath.isFirstChunk();

    srv.request.agent_id = agentID;
    srv.request.next_chunk = next_chunk;
    srv.request.start_node = start_position;
    srv.request.end_node = end_position;

    ros::service::waitForService(srv_name.c_str());
	if (client.call(srv)){
		ROS_INFO("[%s]: Successfully started to calculate path!", agentID.c_str());

        // ROS_INFO("[%s]: Path --> StartPoint (x=%f, y=%f)", agentID.c_str(), srv.response.new_path_nodes.begin()->x, srv.response.new_path_nodes.begin()->y);

		double min_time_take = calculateTimeFromDistanceAndVelocity(srv.response.main_path_length, robotConfig.min_linear_vel);

		if(min_time_take >= 0) {
			if(min_time_take == 0) {
				ROS_ERROR("[%s]: Path length is 0: %s!", agentID.c_str(), srv_name.c_str());
			}

			int last_value_index = srv.response.new_path_nodes.size()-1;

			// TODO: examine the Plan class to see how to construct a Plan object to have the robot navigate when executed.
	        Plan plan = Plan();
	        // TODO: for the last path loading, unloading, charging has to be considered for the robot to position accordingly.
	        // if (srv.response.is_last_chunk) {
	        //		currentPath.setDone(true);
	        // 		std::vector<geometry_msgs::Point> path_to_goal;
	        //      path_to_goal=srv.response.new_path_nodes;
	        //      path_to_goal.push_back(currentPath.getEndPosition());
			  	// TODO: The path is only computed by path planner to the ApproachPoint. After the approach point, there is a hand coded motion starts(going straight to the tray)
	        // From there to the tray the robot drives just straight.
	        // E.g. An intermediate point might be added in this straight part to verify
	        // a smoother ride with the MotionPlanner
        	// You can check if the path is a LOAD, UNLOAD or CHARGING path as below
          //		  switch(currentPath.getPathGoal()) {
        	//          // TODO: fill the strategies for LOAD, UNLOAD or CHARGING, if any different.
        	//			case GOAL::LOAD: {} break;
        	//			case GOAL::UNLOAD: {
	        //                  ROS_INFO("[%s]: Generating an unload plan for the last main-path chunk.", agentID.c_str());
	        //					// BELOW IS AN EXAMPLE OF HOW A PLAN OBJECT IS CONSTRUCTED !
	        //                  plan = Plan(agentID,
	        //                  *motionPlanner,
	        //                  *gripper,
	        //                  false,
	        //                  start_position,
	        //                  path_to_goal,
	        //                  currentPath.getDirectionPoint(),
	        //                  currentPath.getDriveBackPoint(),
	        //                  min_time_take);
	        //              }
	        //              break;
	        //				case GOAL::CHARGE: {} break;
	        // } else {
                // Constructor for a plan to drive to a certain goal position.
            //  plan = Plan(agentID, *motionPlanner, start_position, srv.response.new_path_nodes,
            //  srv.response.new_path_nodes[last_value_index], length);
            // }
	        // The rest of the variable assignments, motion and obstacle detector initiators/enablers should stay for the functionality
            currentPlan = plan;
			this->motionPlanner->enable(true);
			this->motionPlanner->start();
			this->obstacleDetection->enable(true);

            if (currentPath.isFirstChunk()) {
                // After getting the first chunk of the path set first chunk value of the currentPath to false
                currentPath.setFirstChunk(false);
            }
            ROS_INFO("[%s]: Next path chunk was generated.", agentID.c_str());
            return true;
		}
		else {
			ROS_WARN("[%s]: No path found: %s!", agentID.c_str(), srv_name.c_str());
		}
	}
	else {
		ROS_ERROR("[%s]: Failed to call service %s!", agentID.c_str(), srv_name.c_str());
	}

	return false;
}

void Agent::setCurrentPath(Path new_path) {
    currentPath = new_path;
}

bool Agent::assignTask(auto_smart_factory::AssignTask::Request &req,
		auto_smart_factory::AssignTask::Response &res) {
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


            Path path_to_input_tray(agentID, task_id, GOAL::LOAD, position, input_drive_point, input_approach_point, input_tray_position, input_drive_back_point, true);

            //    When Robot arrived at input_tray it will start traveling from input tray to output tray
            Path path_to_storage_tray(agentID, task_id, GOAL::UNLOAD, input_drive_back_point,
                        storage_drive_point, storage_approach_point, storage_tray_position, storage_drive_back_point, false);

            // 2- Set currentPath
            setCurrentPath(path_to_input_tray);
            hasDriven = true;

            // 3- Add the remaining paths to the pathsStack
            pathsStack.push(path_to_storage_tray);

			ROS_INFO("[%s]: Task %i successfully assigned!", agentID.c_str(), req.task_id);
			initialTimeOfCurrentTask = ros::Time::now().toSec();
			ROS_INFO("assignTask %s %.2f %i", agentID.c_str(), initialTimeOfCurrentTask, task_id);
			setState(false);     //Set to non idle if a task is assigned
			res.success = getTheNextPath();
		} else {
			ROS_WARN("[%s]: Is busy! - Task %i has not been assigned!",
					agentID.c_str(), req.task_id);
			res.success = false;
		}
	} catch (std::out_of_range &e) {
		// task does not exist
		ROS_ERROR("[%s]: Attempted to assign inexistent task (specified id: %d)",
				agentID.c_str(), req.task_id);
		res.success = false;
	}
  	return res.success;
}

auto_smart_factory::Tray Agent::getTray(unsigned int tray_id) {
	for(int i=0; i < warehouseConfig.trays.size(); i++)
		if(tray_id == warehouseConfig.trays[i].id)
			return warehouseConfig.trays[i];
	ROS_ERROR("[%s]: Tray with id %u inexistent!",
			agentID.c_str(), tray_id);
}



void Agent::poseCallback(const geometry_msgs::PoseStamped& msg){
	position = msg.pose.position;
	orientation = msg.pose.orientation;
	if(this->motionPlanner->isEnabled()) {
		this->obstacleDetection->enable(true);
		this->motionPlanner->update(position, asin(orientation.z));
	} else
		this->obstacleDetection->enable(false);
}

void Agent::laserCallback(const sensor_msgs::LaserScan& msg) {
	ROS_DEBUG("[%s]: Laser callback: angle_min=%f, angle_max=%f, ranges_size=%lu",
			agentID.c_str(), msg.angle_min, msg.angle_max, msg.ranges.size());
	if(this->obstacleDetection->isEnabled())
		this->obstacleDetection->update(position, asin(orientation.z), msg);
}

void Agent::batteryCallback(const std_msgs::Float32& msg) {
	batteryLevel = msg.data;
	ROS_DEBUG("[%s]: Battery Level: %f!", agentID.c_str(), batteryLevel);
}

double Agent::calculateTimeFromDistanceAndVelocity(double distance, double velocity) {
    return distance/velocity;
}

std::string Agent::GoalToString (GOAL goal){
	if (goal == GOAL::LOAD){
		return "LOAD";
	}
	else if(goal == GOAL::UNLOAD){
		return "UNLOAD";
	}
	else if(goal == GOAL::CHARGE){
		return "CHARGE";
	}
	else if(goal == GOAL::IDLE){
	return "IDLE";
	}
}

float Agent::randomFloat(float min, float max){
	srand((unsigned int) time(NULL));
	float randNum = ((float)rand()/(float)(RAND_MAX)) * (max - min) + min;
	ROS_INFO("Random: %f", randNum);
	return randNum;
}

std::string Agent::getAgentID(){
    return agentID;
}

geometry_msgs::Point Agent::getCurrentPosition(){
	return position;
}

geometry_msgs::Quaternion Agent::getCurrentOrientation(){
	return orientation;
}
