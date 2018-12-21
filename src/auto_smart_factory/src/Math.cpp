#include <time.h>
#include <include/Math.h>

#include "Math.h"
#include "agent/path_planning/Point.h"

void Math::initRandom() {
	initRandom(static_cast<unsigned long>(time(nullptr)));
}

void Math::initRandom(unsigned long seed) {
	srand(static_cast<unsigned int>(seed));
}

float Math::getRandom(float min, float max) {
	// Using rand instead of C++ random library is totally sufficient and faster
	float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	return r * (max - min) + min;
}

float Math::dotProduct(const Point& v1, const Point& v2) {
	return v1.x*v2.x + v1.y*v2.y;
}

float Math::crossProduct(const Point& v1, const Point& v2) {
	return v1.x*v2.y - v1.y*v2.x;
}

Point Math::rotateVector(const Point& v, float angle) {
	angle *= TO_RAD;

	Point r;
	r.x = v.x * std::cos(angle) - v.y * std::sin(angle);
	r.y = v.x * std::sin(angle) + v.y * std::cos(angle);

	return r;
}

float Math::getDistance(const Point& v1, const Point& v2) {
	return sqrtf((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
}

float Math::getLength(const Point& v) {
	return sqrtf(v.x * v.x + v.y * v.y);
}

float Math::getDistanceSquared(const Point& v1, const Point& v2) {
	return (v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y);
}

/*bool Math::areLineSegmentsParallel(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End) {
	Point v1 = l1End - l1Start;
	Point v2 = l2End - l2Start;
	return std::fabs(v1.x*v2.y - v1.y*v2.x) < EPS;
}*/

/*bool Math::doLineSegmentsIntersect(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End) {
	// See https://stackoverflow.com/a/100165
	float s1_x = l1End.x - l1Start.x;
	float s1_y = l1End.y - l1Start.y;
	float s2_x = l2End.x - l2Start.x;
	float s2_y = l2End.y - l2Start.y;

	float det = -s2_x * s1_y + s1_x * s2_y;

	// Parallel or colinear
	if(std::fabs(det) < EPS) {
		// Check if parallel
		if(!areLineSegmentsParallel(l1Start, l1End, l1Start, l2Start)) {
			return false;
		}

		Point dir = l1End - l1Start;

		float dotDirDir = dotProduct(dir, dir);
		float t1 = 0.f;
		float t2 = 1.f;
		Point tol2Start = l2Start - l1Start;
		Point tol2End = l2End - l1Start;
		float t3 = dotProduct(tol2Start, dir) / dotDirDir;
		float t4 = dotProduct(tol2End, dir) / dotDirDir;

		if(t4 < t3) {
			float tmp = t3;
			t3 = t4;
			t4 = tmp;
		}

		return (t2 - t3 >= 0 && t4 - t1 >= 0);
	}

	float s, t;
	s = (-s1_y * (l1Start.x - l2Start.x) + s1_x * (l1Start.y - l2Start.y)) / det;
	t = ( s2_x * (l1Start.y - l2Start.y) - s2_y * (l1Start.x - l2Start.x)) / det;

	return s > 0.f && s < 1.f && t > 0.f && t < 1.f;
}*/

/*bool Math::doesLineSegmentIntersectRectangle(const Point& lStart, const Point& lEnd, Rectangle& rectangle) {
	bool intersectsAxisAligned = doesLineSegmentIntersectAxisAlignedRectangle(lStart, lEnd, rectangle);

	if(rectangle.getIsAxisAligned()) {
		return intersectsAxisAligned;
	} else {
		if(!intersectsAxisAligned) {
			return false;
		}

		return (doLineSegmentsIntersect(lStart, lEnd, rectangle.getPointsInflated()[0], rectangle.getPointsInflated()[1]) ||
		        doLineSegmentsIntersect(lStart, lEnd, rectangle.getPointsInflated()[1], rectangle.getPointsInflated()[2]) ||
		        doLineSegmentsIntersect(lStart, lEnd, rectangle.getPointsInflated()[2], rectangle.getPointsInflated()[3]) ||
		        doLineSegmentsIntersect(lStart, lEnd, rectangle.getPointsInflated()[3], rectangle.getPointsInflated()[0]));
	}
}*/

float Math::getRotation(const Point& v) {
	if(v.x != 0 || v.y != 0) {
		double d = (atan2(v.y, v.x) * TO_DEG);

		if(d < 0) {
			d+= 360;
		}
		return static_cast<float>(d);
	}
	return 0;
}

float Math::toRad(float angle) {
	return angle * TO_RAD;
}

float Math::toDeg(float angle) {
	return angle * TO_DEG;
}

/*bool Math::doesLineSegmentIntersectAxisAlignedRectangle(const Point& lStart, const Point& lEnd, Rectangle& rectangle) {
	// Find min and max X for the segment
	float minX = lStart.x;
	float maxX = lEnd.x;

	if(lStart.x > lEnd.x) {
		minX = lEnd.x;
		maxX = lStart.x;
	}

	// Find the intersection of the segment's and rectangle's x-projections
	if(minX < rectangle.getMinXInflated()) {
		minX = rectangle.getMinXInflated();
	}
	if(maxX > rectangle.getMaxXInflated()) {
		maxX = rectangle.getMaxXInflated();
	}

	// If their projections do not intersect return false
	if(minX > maxX) {
		return false;
	}

	// Find corresponding min and max Y for min and max X we found before
	float minY = lStart.y;
	float maxY = lEnd.y;

	float dx = lEnd.x - lStart.x;

	if(std::fabs(dx) > 0.0000001f) {
		float a = (lEnd.y - lStart.y) / dx;
		float b = lStart.y - a * lStart.x;
		minY = a * minX + b;
		maxY = a * maxX + b;
	}

	if(minY > maxY) {
		float tmp = maxY;
		maxY = minY;
		minY = tmp;
	}

	// Find the intersection of the segment's and rectangle's y-projections
	if(maxY > rectangle.getMaxYInflated()) {
		maxY = rectangle.getMaxYInflated();
	}

	if(minY < rectangle.getMinYInflated()) {
		minY = rectangle.getMinYInflated();
	}

	// If Y-projections do not intersect return false
	if(minY > maxY) {
		return false;
	}

	return true;
}*/

/*float Math::projectPointOnLineSegment(const Point& lStart, const Point& lEnd, const Point& point) {
	Point line = lEnd - lStart;
	Point startToPoint = point - lStart;
	float lengthSquared = getDistanceSquared(lStart, lEnd);
	return dotProduct(line, startToPoint) / lengthSquared;
}*/

float Math::getDistanceToLineSegment(const Point& lStart, const Point& lEnd, const Point& point) {
	Point line = lEnd - lStart;
	Point startToPoint = point - lStart;
	float lengthSquared = getDistanceSquared(lStart, lEnd);
	float t = dotProduct(line, startToPoint) / lengthSquared;

	if(t <= 0) {
		return getDistance(lStart, point);
	} else if(t >= 1) {
		return getDistance(lEnd, point);
	} else {
		return getDistance(point, lStart + t * line);
	}
}

int Math::getDirectionToLineSegment(const Point& lStart, const Point& lEnd, const Point& point) {
	Point nEnd = lEnd - lStart;
	Point nPoint = point - lStart;
	float cP = crossProduct(nEnd, nPoint);

	if(cP > 0)
		return 1;
	
	if(cP < 0)
		return -1;
	
	return 0;
}

float Math::getAngleBetweenVectors(const Point& v1, const Point& v2) {
	return acos(dotProduct(v1, v2) / (getLength(v1)*getLength(v2)));
}

Point Math::getVectorFromOrientation(float o) {
	return Point(cos(o), sin(o));
}

float Math::getAngleMedian(float a1, float a2) {
	float diff = a2 - a1;

	if(diff > 180) {
		diff = diff - 360;
	} else if(diff < -180){
		diff = diff + 360;
	}

	return a1 + diff/2.f;
}

float Math::lerp(float start, float end, float alpha) {
	return start + (end-start) * alpha;
}

float Math::clamp(float value, float min, float max) {
	return std::max(min, std::min(max, value));
}

float Math::getAngleDifferenceInDegree(float source, float target) {
	float angle = target - source;

	if(angle > 180) {
		angle = angle - 360;
	} else if(angle < -180){
		angle = angle + 360;
	}

	return angle;
}

float Math::getAngleDifferenceInRad(float source, float target) {
	float angle = target - source;

	if(angle > M_PI) {
		angle = angle - 2*M_PI;
	} else if(angle < -M_PI){
		angle = angle + 2*M_PI;
	}

	return angle;
}