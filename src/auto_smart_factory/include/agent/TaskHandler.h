#ifndef AGENT_TASKHANDLER_H_
#define AGENT_TASKHANDLER_H_

#include<vector>
#include<string>

#include "ros/ros.h"
#include "agent/Task.h"
#include "auto_smart_factory/TaskAnnouncement.h"
#include "auto_smart_factory/TaskRating.h"

class TaskHandler
{
  public:
    explicit TaskHandler(std::string agentId, ros::Publisher* scorePublish);

    void announcementCallback(const auto_smart_factory::TaskAnnouncement &taskAnnouncement);
    void publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId);

    virtual ~TaskHandler();

  private:
    Task* currentTask;
    std::vector<Task*> queue;

    std::string agentId;

    ros::Publisher* scorePublisher;
};

#endif /* AGENT_TASKHANDLER_H_ */