#ifndef AGENT_PIDCONTROLLER_H_
#define AGENT_PIDCONTROLLER_H_

#include <cmath>
#include "ros/ros.h"

class PidController
{
  public:
    PidController(double targetValue, double proportionalGain, double integralGain, double derivativeGain);

    void updateTargetValue(double targetValue);
    double calculate(double currentValue, double currentTime);
    void reset();

    virtual ~PidController() = default;

  private:
    double targetValue;

    double proportionalGain;
    double integralGain;
    double derivativeGain;

    double integralSum = 0.0;

    double previousError = 0.0;
    double previousTime = -1.0;

};

#endif /* AGENT_PIDCONTROLLER_H_ */