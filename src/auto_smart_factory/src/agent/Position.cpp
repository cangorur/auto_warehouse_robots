#include "agent/Position.h"

Position::Position(double x, double y, double o, ros::Time t)
		:
		x(x),
		y(y),
		o(o),
        t(t) {
}

Position::Position()
		:
		x(0),
		y(0),
		o(0),
        t(ros::Time::now()) {
}

double Position::getAngle(Position* target) {
    return atan2((target->y - y), (target->x - x));
}

double Position::getDistance(Position* target) {
    return sqrt((pow(target->y -y, 2.0) + (pow(target->x - x, 2.0))));
}

Position::~Position() {
}