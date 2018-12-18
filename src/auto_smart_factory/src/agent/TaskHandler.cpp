#include "agent/TaskHandler.h"

TaskHandler::TaskHandler(std::string agentId, ros::Publisher scorePub) 
        : 
    agentId(agentId)
    scorePublisher(scorePub)
{
}

void announcementCallback(auto_smart_factory::TaskAnnouncement &taskAnnouncement) {
    // handle task announcement
    // get score
    publishScore(15.5);
}

void publishScore(double requestId, double score) {
    auto_smart_factory::TaskRating msg;
    // add trays, robot id, ...
    scoreMessage.robot_id = agentId;
    scoreMessage.request_id = requestId;
    scoreMessage.score = score;
    scorePublisher->publish(msg);
}

TaskHandler::~TaskHandler() = default;
