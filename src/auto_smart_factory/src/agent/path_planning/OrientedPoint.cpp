#include <agent/path_planning/OrientedPoint.h>

OrientedPoint::OrientedPoint(float x, float y, float o) :
	x(x),
	y(y),
	o(o) {
}

OrientedPoint::OrientedPoint() :
		x(0),
		y(0),
		o(0) {
}

OrientedPoint::~OrientedPoint() {
}