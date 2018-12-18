#include "agent/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher* scorePub) 
        : 
    agentId(agentId),
    scorePublisher(scorePub)
{
}

void TaskHandler::announcementCallback(const auto_smart_factory::TaskAnnouncement &taskAnnouncement) {
    // handle task announcement
    // get score
    publishScore(0, 15.5);
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
