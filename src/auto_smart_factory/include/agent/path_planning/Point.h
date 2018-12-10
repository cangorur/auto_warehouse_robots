#ifndef AGENT_POINT_H_
#define AGENT_POINT_H_

class Point {
public:
	explicit Point();
	explicit Point(float x, float y);
	virtual ~Point() = default;

	float x;
	float y;

	// Overloaded functions
	/*Point operator -(const Point& right);
	Point operator +(const Point& right);
	Point operator *(float factor);

	Point& operator -=(const Point& right);
	Point& operator +=(const Point& right);*/
};

void test(int a);

Point operator -(const Point& left, const Point& right);
Point operator +(const Point& left, const Point& right);
Point operator *(float factor, const Point& left);
Point operator *(const Point& left, float factor);

Point& operator -=(const Point& left, const Point& right);
Point& operator +=(const Point& left, const Point& right);
  

#endif /* AGENT_POINT_H_ */