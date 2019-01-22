#ifndef PROTOTYPE_RECTANGLE_HPP
#define PROTOTYPE_RECTANGLE_HPP

#include "agent/path_planning/Point.h"

class Rectangle {
private:	
	Point pos;
	Point size;
	float rotation;

	float startTime;
	float endTime;
	
	// Todo add visualisation color
	
	// Faster physic processing
	Point pointsInflated[4];
	bool isAxisAligned;
	float minXInflated, maxXInflated, minYInflated, maxYInflated;

public:
	Rectangle(Point pos, Point size, float rotation);
	Rectangle(Point pos, Point size, float rotation, float startTime, float endTime);
	
	bool isInsideInflated(const Point& point) const;
	const Point* getPointsInflated() const;
	Point getPosition() const;
	Point getSize() const;
	float getRotation() const;
	
	bool getIsAxisAligned() const;
	float getMinXInflated() const;
	float getMaxXInflated() const;
	float getMinYInflated() const;
	float getMaxYInflated() const;

	bool doesOverlapTimeRange(float start, float end) const;
	float getStartTime() const;
	float getEndTime() const;
	float getFreeAfter() const;

private:
	bool isInsideAxisAlignedInflated(const Point& point) const;	

};


#endif //PROTOTYPE_RECTANGLE_HPP
