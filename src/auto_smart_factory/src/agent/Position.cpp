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
		x(0.0),
		y(0.0),
		o(0.0),
        t(ros::Time::now()) {
}

double Position::getAngle(Position* target) {
    return atan2((target->y - y), (target->x - x));
}

double Position::getDistance(Position* target) {
    return sqrt((pow(target->y -y, 2.0) + (pow(target->x - x, 2.0))));
}

void Position::update(double nx, double ny, double no, ros::Time nt) {
	x = nx;
	y = ny;
	o = no;
	t = nt;
}

Position::~Position() {
}