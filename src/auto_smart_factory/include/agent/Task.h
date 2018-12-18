#ifndef AGENT_TASK_H_
#define AGENT_TASK_H_

#include "ros/ros.h"

class Task
{
  public:
    explicit Task();

    virtual ~Task();

  private:
    unsigned int id;
    
};

#endif /* AGENT_TASK_H_ */