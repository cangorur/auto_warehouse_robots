#ifndef AGENT_POSITION_H_
#define AGENT_POSITION_H_

#include <math.h>
#include "ros/ros.h"

class Position {
public:
	explicit Position();

	explicit Position(double x, double y, double o, ros::Time t);

	virtual ~Position();

    double getAngle(Position* target);

    double getDistance(Position* target);

	double x;
	double y;
	double o;
    ros::Time t;
};

#endif /* AGENT_POSITION_H_ */