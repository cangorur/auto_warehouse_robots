#include <agent/Path.h>

Path::Path(std::string agent_id, int task_id,  GOAL path_goal, geometry_msgs::Point start_position, geometry_msgs::Point end_position,
    geometry_msgs::Point approach_point, geometry_msgs::Point direction_point, geometry_msgs::Point drive_back_point, bool idle) {

    agentID = agent_id;
    taskID = task_id;
    startPosition = start_position;
    endPosition = end_position;
    this->idle = idle;
    pathGoal = path_goal;
    done = false;
    firstChunk = true;
    directionPoint = direction_point;
    driveBackPoint = drive_back_point;
    approachPoint = approach_point;
}

Path::Path(){
}

Path::~Path() {
}

bool Path::isDone() {
    return done;
}

void Path::setDone(bool done) {
    this->done = done;
}

bool Path::isFirstChunk() {
    return firstChunk;
}

void Path::setFirstChunk(bool firstChunk) {
    this->firstChunk = firstChunk;
}

int Path::getTaskId() {
    return taskID;
}

bool Path::isIdle() {
    return idle;
}

GOAL Path::getPathGoal() {
    return pathGoal;
}

geometry_msgs::Point Path::getDirectionPoint() {
    return directionPoint;
}

geometry_msgs::Point Path::getDriveBackPoint() {
    return driveBackPoint;
}

geometry_msgs::Point Path::getApproachPoint() {
    return approachPoint;
}

geometry_msgs::Point Path::getEndPosition() {
    return endPosition;
}

geometry_msgs::Point  Path::getStartPosition() {
    return startPosition;
}