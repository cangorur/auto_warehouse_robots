#ifndef AGENT_POINT_H_
#define AGENT_POINT_H_

#include "geometry_msgs/Point.h"
#include "OrientedPoint.h"

class Point {
public:
	explicit Point();
	explicit Point(float x, float y);
	explicit Point(geometry_msgs::Point point);
	explicit Point(OrientedPoint point);
	virtual ~Point() = default;

	float x;
	float y;
};

// Overloaded operators
Point operator -(const Point& right);
Point operator -(const Point& left, const Point& right);
Point operator +(const Point& left, const Point& right);
Point operator *(float factor, const Point& left);
Point operator *(const Point& left, float factor);
Point operator /(const Point& left, float factor);
bool operator ==(const Point& left, const Point& right);

Point& operator -=(const Point& left, const Point& right);
Point& operator +=(const Point& left, const Point& right);
  

#endif /* AGENT_POINT_H_ */