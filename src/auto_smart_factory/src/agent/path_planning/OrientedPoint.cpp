#include <agent/path_planning/OrientedPoint.h>

OrientedPoint::OrientedPoint(double x, double y, double o)	:
		x(x),
		y(y),
		o(o) {
}

OrientedPoint::OrientedPoint():
		x(0),
		y(0),
		o(0) {
}
