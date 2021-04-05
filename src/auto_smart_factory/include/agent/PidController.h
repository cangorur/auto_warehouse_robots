#ifndef AGENT_PIDCONTROLLER_H_
#define AGENT_PIDCONTROLLER_H_

#include <cmath>
#include "ros/ros.h"

/*
 * This class implements a standard pid controller without any more magic (see wikipedia for: What is a pid controller?)
 */
class PidController
{
  public:
    /*
     * Initializie the pid controller with the target value and 
     * the multipliers for the proportional, integral and derivate errors
     */
    PidController(double targetValue, double proportionalGain, double integralGain, double derivativeGain);

    /* 
     * Change the targetValue during operation 
     * If derivate and integral errors are used, this should be used carefully
     * @param targetValue the new target value
     */
    void updateTargetValue(double targetValue);

    /*
     * Calculate the new ouput for the current input value 
     * @param currentValue the current value which regulates the output value
     * @param currentTime the current time used for the integral and derivate errors
     * @output double the new input value for the system to correct the output towards the target value
     */
    double calculate(double currentValue, double currentTime);

    /*
     * Reset time, previous error and integral sum
     */
    void reset();

    /*
     * virtually destructs the pid controller
     */
    virtual ~PidController() = default;

  private:
    // the value the pid controller tries to maintain
    double targetValue;

    // the multipliers for the proportional, integral and derivative errors
    double proportionalGain;
    double integralGain;
    double derivativeGain;

    // contains the sum to calculate the intergral error
    double integralSum = 0.0;

    // previous error and time for the derivative and integral error
    double previousError = 0.0;
    double previousTime = -1.0;

};

#endif /* AGENT_PIDCONTROLLER_H_ */