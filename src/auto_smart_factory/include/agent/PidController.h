#ifndef AGENT_PIDCONTROLLER_H_
#define AGENT_PIDCONTROLLER_H_

#include "ros/ros.h"
#include "agent/Position.h"
#include "geometry_msgs/Twist.h"

#define PI 3.141592
#define F_KP 2.58  	// P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  	// S constant for PSD translation controller
#define R_KP 2.0  	// P constant for PSD rotation controller
#define R_KD 0.1  	// D constant for PSD rotation controller
#define R_KI 0.0  	// S constant for PSD rotation controller

class PidController {
public:
	PidController(ros::Publisher pub, double posTolerance, double angleTolerance, double maxSpeed, double maxAngleSpeed);

	void setTarget(double distance, double angle);

	void publishVelocity(double speed, double angle);

	void update(Position* pos);

	bool targetReached(Position* current);

	double calculatePSD(Position* current, double currentValue, double lastValue, double referenceValue, double kP, double kD, double kS, double* sum);

	virtual ~PidController();


	Position* start;
	Position* last;
	double maxSpeed;
	double maxAngleSpeed;
	double posTolerance;
	double angleTolerance;
	double targetDistance;
	double targetAngle;
	double sumDistance;
	double sumAngle;
	int iterations;
	ros::Publisher pubVelocity;

};

#endif /* AGENT_PIDCONTROLLER_H_ */