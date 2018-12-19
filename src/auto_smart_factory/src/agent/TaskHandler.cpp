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
        ROS_WARN("[TaskHandler - %s] received TaskAnnouncement with %d start Trays and %d end trays", agentId.c_str(), tA.start_ids.size(), tA.end_ids.size());
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
