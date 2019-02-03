#include <agent/path_planning/Point.h>
#include <include/agent/path_planning/Point.h>
#include <iostream>


Point::Point(double x, double y) :
		x(x),
		y(y) { }

Point::Point(geometry_msgs::Point point) :
		x(point.x),
		y(point.y) { }

Point::Point() :
		x(0),
		y(0) { }

Point::Point(OrientedPoint point) :
		x(point.x),
		y(point.y) { }

// Overloaded operators
Point operator -(const Point& right) {
	return Point(-right.x, -right.y);
}

Point operator -(const Point& left, const Point& right) {
	return Point(left.x - right.x, left.y - right.y);
}

Point operator +(const Point& left, const Point& right) {
	return Point(left.x + right.x, left.y + right.y);
}

Point& operator -=(Point& left, const Point& right) {
	left.x -= right.x;
	left.y -= right.y;

	return left;
}

Point& operator +=(Point& left, const Point& right) {
	left.x += right.x;
	left.y += right.y;

	return left;
}

Point operator *(double left, const Point& right) {
	return Point(left * right.x, left * right.y);
}

Point operator *(const Point& left, double factor) {
	return Point(left.x * factor, left.y * factor);
}

Point operator /(const Point& left, double factor) {
	return Point(left.x / factor, left.y / factor);
}

bool operator ==(const Point& left, const Point& right) {
	return left.x == right.x && left.y == right.y;
}


