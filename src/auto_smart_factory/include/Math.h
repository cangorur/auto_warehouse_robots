#ifndef PROJECT_MATH_H
#define PROJECT_MATH_H

#include <cmath>
#include <random>

#include "agent/path_planning/Point.h"
#include "agent/path_planning/Rectangle.h"

#define TO_RAD 0.01745329252f
#define TO_DEG 57.2957795131f
#define PI 3.14159265359f
#define EPS 1e-6

// Defined as 0.25 + error margin
#define ROBOT_RADIUS 0.275f
#define APPROACH_DISTANCE 0.1f

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
	static double getRandom(double min, double max);

	static double toRad(double angle);
	static double toDeg(double angle);
	static double clamp(double value, double min, double max);
	static double lerp(double start, double end, double alpha);	
	
	static double dotProduct(const Point& v1, const Point& v2);
	static double crossProduct(const Point& v1, const Point& v2);
	static Point rotateVector(const Point& v, double angle);
	static double getRotationInDeg(const Point& v);
	static double getDistance(const Point& v1, const Point& v2);
	static double getLength(const Point& v);
	static double getDistanceSquared(const Point& v1, const Point& v2);
	
	static bool areLineSegmentsParallel(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End);
	static bool doLineSegmentsIntersect(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End);
	static bool doesLineSegmentIntersectRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle);
	static bool isPointInRectangle(const Point& p, const Rectangle& rectangle);

	static double projectPointOnLineSegment(const Point& lStart, const Point& lEnd, const Point& point);
	static double getDistanceToLineSegment(const Point& lStart, const Point& lEnd, const Point& point);
	static int getDirectionToLineSegment(const Point& lStart, const Point& lEnd, const Point& point);

	static double getDistanceToLine(const Point& lStart, const Point& lEnd, const Point& point);

	static double getAngleBetweenVectors(const Point& v1, const Point &v2);

	static Point getVectorFromOrientation(double o);
	static double getOrientationFromVector(const Point& v);
	
	static double getAngleMedian(double a1, double a2);
	static double getAngleDifferenceInDegree(double source, double target);
	static double getAngleDifferenceInRad(double source, double target);

	static double normalizeRad(double angle);
	static double normalizeDegree(double angle);

private:
	static bool doesLineSegmentIntersectAxisAlignedRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle);
	static bool doesLineSegmentIntersectNonAxisAlignedRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle);
};

#endif //PROJECT_MATH_H
