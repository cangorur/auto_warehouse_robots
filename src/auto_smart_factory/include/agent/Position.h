#ifndef AGENT_POSITION_H_
#define AGENT_POSITION_H_

#include <cmath>
#include "ros/ros.h"

class Position {
public:
	explicit Position();

	explicit Position(double x, double y, double o, ros::Time t);

	virtual ~Position();

    double getAngle(Position* target);

    double getDistance(Position* target);

	void update(double nx, double ny, double no, ros::Time nt);

	double x;
	double y;
	double o;
    ros::Time t;
};

#endif /* AGENT_POSITION_H_ */