#include <agent/Plan.h>

Plan::Plan() {
	done = true;
}

Plan::Plan(std::string agent_id, MotionPlanner &motion_planner, geometry_msgs::Point start_position, 
		std::vector<geometry_msgs::Point> path, geometry_msgs::Point end_direction_point, 
		double length,bool drive_backwards) {
	agentID = agent_id;
	motionPlanner = &motion_planner;
	startPosition = start_position;
	this->length = length;

	SubPlan plan(path, end_direction_point, drive_backwards);
	subplans.push_back(plan);
}

Plan::Plan(std::string agent_id, MotionPlanner &motion_planner, Gripper &gripper, unsigned int storage_id,
		geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> path_to_load, 
		geometry_msgs::Point load_direction_point, 
		std::vector<geometry_msgs::Point> path_to_unload, 
		geometry_msgs::Point unload_direction_point, double length) {
	agentID = agent_id;
	motionPlanner = &motion_planner;
	this->gripper = &gripper;
	startPosition = start_position;
	storageID = storage_id;
	loadDirectionPoint = load_direction_point;
	unloadDirectionPoint = unload_direction_point;
	this->length = length;

	SubPlan planToLoad(path_to_load, loadDirectionPoint, false);
	subplans.push_back(planToLoad);

	SubPlan load(true, path_to_load[path_to_load.size() - 1], loadDirectionPoint);
	subplans.push_back(load);

	std::vector<geometry_msgs::Point> driveBackPath;
	driveBackPath.push_back(path_to_unload[0]);
	SubPlan driveBack(driveBackPath, loadDirectionPoint, true);
	subplans.push_back(driveBack);

	path_to_unload.erase(path_to_unload.begin());
	SubPlan planToUnload(path_to_unload, unloadDirectionPoint, false);
	subplans.push_back(planToUnload);

	SubPlan unload(false, path_to_unload[path_to_unload.size() - 1], unloadDirectionPoint);
	subplans.push_back(unload);
}

Plan::Plan(std::string agent_id, MotionPlanner &motion_planner, Gripper &gripper, bool load_action,
		geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> path, geometry_msgs::Point direction_point,
		geometry_msgs::Point drive_back_point, double length) {

	agentID = agent_id;
	motionPlanner = &motion_planner;
	this->gripper = &gripper;
	startPosition = start_position;
	this->length = length;

     /*std::ostringstream st;
     st << "in Plan::Plan load action " << load_action << std::endl;
        for (auto it = path.begin(); it != path.end(); ++it){
            st << "x: " << it->x << ", y: " << it->y << std::endl;
        }
     st << "direction POint x: " << direction_point.x << ", y: " << direction_point.y << std::endl;
     st << "driveback point x: " << drive_back_point.x << ", y: " << drive_back_point.y << std::endl;
        ROS_WARN(st.str().c_str());*/


	if(load_action) {
        loadDirectionPoint = direction_point;
        SubPlan planToLoad(path, loadDirectionPoint, false);
        subplans.push_back(planToLoad);

        SubPlan load(true, path[path.size() - 1], loadDirectionPoint);
        subplans.push_back(load);


    } else {
      //  idlePlan = true;
        unloadDirectionPoint = direction_point;
        SubPlan planToUnload(path, unloadDirectionPoint, false);
        subplans.push_back(planToUnload);
        SubPlan unload(false, path[path.size() - 1], unloadDirectionPoint);
        subplans.push_back(unload);

    }
        std::vector<geometry_msgs::Point> driveBackPath;
	    //driveBackPath.push_back(path[path.size() - 1]);
        driveBackPath.push_back(drive_back_point);   // go backwards to the last point of load path
        SubPlan driveBack(driveBackPath, loadDirectionPoint, true);
        subplans.push_back(driveBack);


}

Plan::Plan(std::string agent_id, MotionPlanner &motion_planner, unsigned int charging_station_id,
		geometry_msgs::Point start_position,
		std::vector<geometry_msgs::Point> path_to_charging_station,
		geometry_msgs::Point charging_direction_point, double length) {

	agentID = agent_id;
	motionPlanner = &motion_planner;
	startPosition = start_position;
	this->length = length;

	isChargin = true;

	SubPlan planToCharge(path_to_charging_station, charging_direction_point, false);
	subplans.push_back(planToCharge);

	SubPlan charge(charging_station_id);
	subplans.push_back(charge);
}

Plan::Plan(std::string agent_id, MotionPlanner &motion_planner, unsigned int charging_station_id,
		geometry_msgs::Point start_position, 
		std::vector<geometry_msgs::Point> path_to_charging_station, 
		geometry_msgs::Point charging_direction_point, 
		std::vector<geometry_msgs::Point> path_to_idle_position, 
		geometry_msgs::Point end_direction_point, double length) {
	agentID = agent_id;
	motionPlanner = &motion_planner;
	startPosition = start_position;
	this->length = length;

	isChargin = true;

	SubPlan planToCharge(path_to_charging_station, charging_direction_point, false);
	subplans.push_back(planToCharge);

	SubPlan charge(charging_station_id);
	subplans.push_back(charge);

	SubPlan planToIdle(path_to_idle_position, end_direction_point, false);
	subplans.push_back(planToIdle);
}

Plan::~Plan() {
}

void Plan::insertCharging(std::string agent_id, unsigned int charging_station_id, 
		std::vector<geometry_msgs::Point> path_to_charging_station, 
		geometry_msgs::Point charging_direction_point, 
		std::vector<geometry_msgs::Point> path_to_goal_position, 
		geometry_msgs::Point goal_direction_point, double length) {
	agentID = agent_id;
	isChargin = true;

	SubPlan planToCharge(path_to_charging_station, charging_direction_point, false);
	//TODO dont overwrite current path but merge both paths
	subplans[currentIndex] = planToCharge;

	SubPlan charge(charging_station_id);
	subplans.insert(subplans.begin()+currentIndex+1, charge);

	SubPlan planToGoal(path_to_goal_position, goal_direction_point, false);
	subplans.insert(subplans.begin()+currentIndex+2, planToGoal);

	//not accurate but sufficient
	this->length += length;	

	//replace current path with path to charging station
	motionPlanner->newPath(startPosition, subplans[currentIndex].getPath(), 
			subplans[currentIndex].getEndDirectionPoint());
	motionPlanner->start();
}

void Plan::insertUn_LoadTask(std::string agent_id,
        bool isLoadPlan,
		geometry_msgs::Point position,
		geometry_msgs::Point direction_point,
		double length) {

    SubPlan planToUn_Load(isLoadPlan, position, direction_point);
    subplans.push_back(planToUn_Load);
    this->length += length;
}

void Plan::insertChargingSimple(unsigned int charging_station_id, double length){

    SubPlan charge(charging_station_id);
	subplans.push_back(charge);
	this->length += length;
}

void Plan::execute(geometry_msgs::Point position, float battery_level) {

	if(done) {
		ROS_WARN("[%s]: Plan already done!", agentID.c_str());
		return;
	}

	//ROS_WARN("Plan::execute");

	//ROS_WARN("%d", start);
	//ROS_WARN("%d", motionPlanner->isDone());

	double acc = motionPlanner->getDriveForwardDistanceAccuracy();
	if(!start && motionPlanner->isDone()) {
		switch(subplans[currentIndex].getType()) {
			case drive: 
				if(getDistance(subplans[currentIndex].getEndPosition(), position)<=acc)
					subplans[currentIndex].setDone(true);
				else {
					//workaround for package loading at false place seems to work
					ROS_ERROR("[%s]: MotionPlanner done, but not at the right Position!", agentID.c_str()); 
					std::vector<geometry_msgs::Point> subpath;
					subpath.push_back(subplans[currentIndex].getEndPosition());
					bool drive_back = false;
					switch(subplans[currentIndex].getType()) {
						case backwards: 
							drive_back = true;
							break;
					}

					SubPlan subplan(subpath, subplans[currentIndex].getEndDirectionPoint(), drive_back);
					subplans[currentIndex] = subplan;
					motionPlanner->newPath(position, subplans[currentIndex].getPath(),
						       	subplans[currentIndex].getEndDirectionPoint());
					motionPlanner->start();

				}
				break;
			case backwards: 
				//TODO maybe the same workaround like with forward driving
				subplans[currentIndex].setDone(true);
				break;
			case charge:{
			//	if(battery_level > 99.0) {
					ROS_DEBUG("[%s] Charging: Checking battery level: %f", agentID.c_str(), battery_level);
					// isChargin = false;
					// // while(!endChargingReservation(   //Legend code
					// // 		subplans[currentIndex].getChargingStationId())) {
					// // 	ROS_WARN("[%s]: Charging reservation has not been ended yet. Waiting for it...!", agentID.c_str());
					// // 	ros::Duration(1.0).sleep();
					// // }
					// subplans[currentIndex].setDone(true);
					//motionPlanner->enable(false);
				}
				break;
		}
	}

	if(start || subplans[currentIndex].isDone()) {
		start = false;
		if(subplans[currentIndex].isDone())
			currentIndex++;
		if(currentIndex < subplans.size()) {
			switch(subplans[currentIndex].getType()) {
				case drive: 
					ROS_INFO("[%s]: SubPlan: drive!", agentID.c_str()); 
					motionPlanner->newPath(startPosition, 
							subplans[currentIndex].getPath(), 
							subplans[currentIndex].getEndDirectionPoint());
					motionPlanner->start();
					break;
				case backwards: {
					/*ROS_INFO("[%s]: SubPlan: backwards!", agentID.c_str());
                    std::ostringstream st;
                    st << "start pos x: " << startPosition.x << " and y " << startPosition.y << std::endl;
                    st << "start pos x: " << subplans[currentIndex].getEndDirectionPoint().x << " and y " << subplans[currentIndex].getEndDirectionPoint().y << std::endl;
                     for (std::vector<geometry_msgs::Point>::iterator it = subplans[currentIndex].getPath().begin();
                             it != subplans[currentIndex].getPath().end(); ++it){

                                st << "x: " << it->x << ", y: " << it->y << std::endl;
                            }
                     ROS_INFO(st.str().c_str());*/

					motionPlanner->newPath(startPosition, 
							subplans[currentIndex].getPath(), 
							subplans[currentIndex].getEndDirectionPoint(), 
							true);
					motionPlanner->start();
					}
					break;
				case load: 
					ROS_INFO("[%s]: SubPlan: load!", agentID.c_str()); 
					if(getDistance(subplans[currentIndex].getEndPosition(), position) 
							> acc)
						ROS_ERROR("[%s]: Not at right position for loading!", 
								agentID.c_str()); 
					else
					if(gripper->loadPackage(true)) {
						ROS_FATAL("load %s %.2f", agentID.c_str(), ros::Time::now().toSec());
						subplans[currentIndex].setDone(true);
					}
					break;
				case unload: 
					ROS_INFO("[%s]: SubPlan: unload!", agentID.c_str()); 
					if(getDistance(subplans[currentIndex].getEndPosition(), position) 
							> acc)
						ROS_ERROR("[%s]: Not at right position for unloading!",
								agentID.c_str()); 
					else
					if(gripper->loadPackage(false)) {
					    ROS_FATAL("unload %s %.2f", agentID.c_str(), ros::Time::now().toSec());
						subplans[currentIndex].setDone(true);
					}
					break;
				case charge:
					ROS_INFO("[%s]: SubPlan: charge!", agentID.c_str()); 
					motionPlanner->enable(false);
					break;
				case unknown:
					ROS_ERROR("[%s]: SubPlanType unknown!", agentID.c_str());
			}
		}		
		else
			done = true;
	}
}

bool Plan::isDone() {
	return done;
}

void Plan::setDone(bool done) {
	this->done = done;
}

unsigned int Plan::getStorageID() {
	return storageID;
}

double Plan::getLength() {
	return length;
}

bool Plan::isDrivingForward() {
	if(subplans.size() == 0 || currentIndex < 0)
		return false;
	switch(subplans[currentIndex].getType()) {
		case drive: 
			return true;
	}
	return false;
}

SubPlan Plan::getCurrentSubPlan() {
	return subplans[currentIndex];
}

bool Plan::isCharging() {
	return isChargin;
}

double Plan::getDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
	return  sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0));
}
