#include <agent/path_planning/Point.h>
#include <include/agent/path_planning/Point.h>
#include <iostream>


Point::Point(float x, float y) :
		x(x),
		y(y) {
}

Point::Point() :
		x(0),
		y(0) {
}

/*Point Point::operator-(const Point& right) {
	return Point(x - right.x, y - right.y);
}

Point Point::operator+(const Point& right) {
	return Point(x + right.x, y + right.y);
}

Point Point::operator*(float factor) {
	return Point(x * factor, y * factor);
}

Point& Point::operator-=(const Point& right) {
	x -= right.x;
	y -= right.y;
	return *this;
}

Point& Point::operator+=(const Point& right) {
	x += right.x;
	y += right.y;
	return *this;
}*/


inline Point operator -(const Point& left, const Point& right) {
	return Point(left.x - right.x, left.y - right.y);
}

inline Point operator +(const Point& left, const Point& right) {
	return Point(left.x + right.x, left.y + right.y);
}

inline Point& operator -=(Point& left, const Point& right) {
	left.x -= right.x;
	left.y -= right.y;

	return left;
}

inline Point& operator +=(Point& left, const Point& right) {
	left.x += right.x;
	left.y += right.y;

	return left;
}

inline Point operator *(float left, const Point& right) {
	return Point(left * right.x, left * right.y);
}

inline Point operator *(const Point& left, float right) {
	return Point(left.x * right, left.y * right);
}

void test(int a) {
	std::cout << "test" << std::endl;
}


