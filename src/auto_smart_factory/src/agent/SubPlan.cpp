#include <agent/SubPlan.h>

SubPlan::SubPlan(std::vector<geometry_msgs::Point> path, geometry_msgs::Point end_direction_point, bool drive_backwards) {
	this->path = path;
	this->endDirectionPoint = end_direction_point;
	if(drive_backwards)
		this->type = backwards;
	else
		this->type = drive;
}

SubPlan::SubPlan(bool do_load, geometry_msgs::Point position, geometry_msgs::Point direction_point) {
	if(do_load)
		this->type = load;
	else
		this->type = unload;
	gripPosition = position;
	endDirectionPoint = direction_point;
}

SubPlan::SubPlan(unsigned int charging_station_id) {
	this->type = charge;
	this->chargingStationID = charging_station_id;
}

SubPlan::~SubPlan() {
}

SubPlanType SubPlan::getType() {
	return type;
}

bool SubPlan::isDone() {
	return done;
}

void SubPlan::setDone(bool done) {
	this->done = done;
}

std::vector<geometry_msgs::Point> SubPlan::getPath() {
	return path;
}

geometry_msgs::Point SubPlan::getEndPosition() {
	if(path.size() > 0)
		return path[path.size() - 1];
	else {
		switch(getType()) {
			case load: 
			case unload: 
			return gripPosition;
		}

		geometry_msgs::Point p;
		p.x = -1;
		p.y = -1;
		p.z = -1;
		return p;
	}
}

geometry_msgs::Point SubPlan::getEndDirectionPoint() {
	return endDirectionPoint;
}

unsigned int SubPlan::getChargingStationId() {
	return chargingStationID;
}
