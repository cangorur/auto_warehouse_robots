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
    publishScore(tA.request_id, 15.5);
}

void TaskHandler::publishScore(unsigned int requestId, double score) {
    auto_smart_factory::TaskRating scoreMessage;
    // add trays, robot id, ...
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.score = score;
    scorePublisher->publish(scoreMessage);
}

TaskHandler::~TaskHandler() = default;
