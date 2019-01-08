#include "agent/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher* scorePub) 
        : 
    agentId(agentId),
    scorePublisher(scorePub)
{
}

void TaskHandler::announcementCallback(const auto_smart_factory::TaskAnnouncement &tA) {
    // handle task announcement
    // get score
    // ROS_WARN("[TaskHandler - %s] sending score!", agentId.c_str());
    if(tA.start_ids.size() > 0 && tA.end_ids.size() > 0){
        publishScore(tA.request_id, 15.5, tA.start_ids.front(), tA.end_ids.front());
    } else {
        ROS_WARN("[TaskHandler - %s] received TaskAnnouncement with %d start Trays and %d end trays", agentId.c_str(), (unsigned int)tA.start_ids.size(), (unsigned int)tA.end_ids.size());
    }
}

void TaskHandler::publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId) {
    auto_smart_factory::TaskRating scoreMessage;
    // add trays, robot id, ...
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.score = score;
    scoreMessage.end_id = endTrayId;
    scoreMessage.start_id = startTrayId;
    scorePublisher->publish(scoreMessage);
}

TaskHandler::~TaskHandler() = default;

void TaskHandler::addTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
				uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath){
    // create new task
    Task t = TransportationTask(id, sourceID, sourcePos, targetID, targetPos, sourcePath, targetPath);

    // add task to list
    queue.push_back(&t);
}

void TaskHandler::nextTask(void){
	if(!queue.empty()){
		currentTask = queue.front();
		queue.pop_front();
	}
}

unsigned int TaskHandler::numberQueuedTasks(void){
	return (unsigned int) queue.size();
}
