#ifndef AGENT_TASKHANDLER_H_
#define AGENT_TASKHANDLER_H_

#include<vector>
#include<string>

#include "ros/ros.h"
#include "agent/Task.h"

class TaskHandler
{
  public:
    explicit TaskHandler(std::string agentId);

    virtual ~TaskHandler();

  private:
    std::vector<Task> queue;

    std::string agentId;
};

#endif /* AGENT_TASKHANDLER_H_ */