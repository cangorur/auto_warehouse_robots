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
    explicit TaskHandler(std::string agentId, ros::Publisher scorePublish);

    void announcementCallback(auto_smart_factory::TaskAnnouncement &taskAnnouncement);
    void publishScore(double score);

    void 

    virtual ~TaskHandler();

  private:
    std::vector<Task> queue;

    std::string agentId;

    ros::Publisher scorePublisher;
};

#endif /* AGENT_TASKHANDLER_H_ */