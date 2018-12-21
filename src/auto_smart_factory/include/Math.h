#ifndef PROJECT_MATH_H
#define PROJECT_MATH_H

#include <cmath>
#include <random>
#include "agent/path_planning/Point.h"
//#include "Rectangle.hpp"

#define TO_RAD 0.01745329252f
#define TO_DEG 57.2957795131f
#define PI 3.14159265359f
#define EPS 1e-6

// Rotation  0  = >
// Rotation 90  = v
// Rotation 180 = <
// Rotation 270 = ^

class Math {
private:
	Math() = default;

public:
	struct PointComparator {
		bool operator()(const Point lhs, const Point rhs) const {
			if(lhs.x != rhs.x) {
				return lhs.x < rhs.x;
			} else {
				return lhs.y < rhs.y;
			}
		}
	};

	/* Random */
	static void initRandom();
	static void initRandom(unsigned long seed);
	static float getRandom(float min, float max);

	static float toRad(float angle);
	static float toDeg(float angle);
	static float clamp(float value, float min, float max);
	static float lerp(float start, float end, float alpha);	
	
	static float dotProduct(const Point& v1, const Point& v2);
	static float crossProduct(const Point& v1, const Point& v2);
	static Point rotateVector(const Point& v, float angle);
	static float getRotation(const Point& v);
	static float getDistance(const Point& v1, const Point& v2);
	static float getLength(const Point& v);
	static float getDistanceSquared(const Point& v1, const Point& v2);
	
	//static bool areLineSegmentsParallel(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End);
	//static bool doLineSegmentsIntersect(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End);
	//static bool doesLineSegmentIntersectRectangle(const Point& lStart, const Point& lEnd, Rectangle& rectangle);
	//static bool doesLineSegmentIntersectAxisAlignedRectangle(const Point& lStart, const Point& lEnd, Rectangle& rectangle);

	//static float projectPointOnLineSegment(const Point& lStart, const Point& lEnd, const Point& point);
	static float getDistanceToLineSegment(const Point& lStart, const Point& lEnd, const Point& point);
	static int getDirectionToLineSegment(const Point& lStart, const Point& lEnd, const Point& point);
	static float getAngleBetweenVectors(const Point& v1, const Point &v2);

	static Point getVectorFromOrientation(float o);
	
	static float getAngleMedian(float a1, float a2);
	static float getAngleDifferenceInDegree(float source, float target);
	static float getAngleDifferenceInRad(float source, float target);
};

#endif //PROJECT_MATH_H
