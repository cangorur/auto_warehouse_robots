#ifndef AGENT_TASKHANDLER_H_
#define AGENT_TASKHANDLER_H_

#include <list>
#include <string>

#include "ros/ros.h"
#include "agent/Task.h"
#include "agent/TransportationTask.h"
#include "agent/ChargingTask.h"
#include "auto_smart_factory/TaskAnnouncement.h"
#include "auto_smart_factory/TaskRating.h"

class TaskHandler
{
  public:
    explicit TaskHandler(std::string agentId, ros::Publisher* scorePublish);

    void announcementCallback(const auto_smart_factory::TaskAnnouncement &taskAnnouncement);
    void publishScore(unsigned int requestId, double score, uint32_t startTrayId, uint32_t endTrayId);

    virtual ~TaskHandler();

    void addTask(unsigned int id, uint32_t sourceID, OrientedPoint sourcePos, 
				uint32_t targetID, OrientedPoint targetPos, Path sourcePath, Path targetPath);

    void nextTask(void);

	unsigned int numberQueuedTasks(void);

  private:
    Task* currentTask = nullptr;
    std::list<Task*> queue;

    std::string agentId;

    ros::Publisher* scorePublisher;
};

#endif /* AGENT_TASKHANDLER_H_ */