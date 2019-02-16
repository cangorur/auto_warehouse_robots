#include <cmath>
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

double Math::getRandom(double min, double max) {
	// Using rand instead of C++ random library is totally sufficient and faster
	double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
	return r * (max - min) + min;
}

double Math::dotProduct(const Point& v1, const Point& v2) {
	return v1.x*v2.x + v1.y*v2.y;
}

double Math::crossProduct(const Point& v1, const Point& v2) {
	return v1.x*v2.y - v1.y*v2.x;
}

Point Math::rotateVector(const Point& v, double angle) {
	angle *= TO_RAD;

	Point r;
	r.x = v.x * std::cos(angle) - v.y * std::sin(angle);
	r.y = v.x * std::sin(angle) + v.y * std::cos(angle);

	return r;
}

double Math::getDistance(const Point& v1, const Point& v2) {
	return std::sqrt((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
}

double Math::getLength(const Point& v) {
	return std::sqrt(v.x * v.x + v.y * v.y);
}

double Math::getDistanceSquared(const Point& v1, const Point& v2) {
	return (v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y);
}

bool Math::areLineSegmentsParallel(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End) {
	Point v1 = l1End - l1Start;
	Point v2 = l2End - l2Start;
	return std::abs(v1.x*v2.y - v1.y*v2.x) < EPS;
}

bool Math::doLineSegmentsIntersect(const Point& l1Start, const Point& l1End, const Point& l2Start, const Point& l2End) {
	// See https://stackoverflow.com/a/100165
	double s1_x = l1End.x - l1Start.x;
	double s1_y = l1End.y - l1Start.y;
	double s2_x = l2End.x - l2Start.x;
	double s2_y = l2End.y - l2Start.y;

	double det = -s2_x * s1_y + s1_x * s2_y;

	// Parallel or colinear
	if(std::fabs(det) < EPS) {
		// Check if parallel
		if(!areLineSegmentsParallel(l1Start, l1End, l1Start, l2Start)) {
			return false;
		}

		Point dir = l1End - l1Start;

		double dotDirDir = dotProduct(dir, dir);
		double t1 = 0.f;
		double t2 = 1.f;
		Point tol2Start = l2Start - l1Start;
		Point tol2End = l2End - l1Start;
		double t3 = dotProduct(tol2Start, dir) / dotDirDir;
		double t4 = dotProduct(tol2End, dir) / dotDirDir;

		if(t4 < t3) {
			double tmp = t3;
			t3 = t4;
			t4 = tmp;
		}

		return (t2 - t3 >= 0 && t4 - t1 >= 0);
	}

	double s, t;
	s = (-s1_y * (l1Start.x - l2Start.x) + s1_x * (l1Start.y - l2Start.y)) / det;
	t = ( s2_x * (l1Start.y - l2Start.y) - s2_y * (l1Start.x - l2Start.x)) / det;

	return s > 0.f && s < 1.f && t > 0.f && t < 1.f;
}

double Math::getRotationInDeg(const Point& v) {
	if(v.x != 0 || v.y != 0) {
		double d = (std::atan2(v.y, v.x) * TO_DEG);

		if(d < 0) {
			d += 360;
		}
		return d;
	}
	return 0;
}

double Math::toRad(double angle) {
	return angle * TO_RAD;
}

double Math::toDeg(double angle) {
	return angle * TO_DEG;
}

double Math::projectPointOnLineSegment(const Point& lStart, const Point& lEnd, const Point& point) {
	Point line = lEnd - lStart;
	Point startToPoint = point - lStart;
	double lengthSquared = getDistanceSquared(lStart, lEnd);
	return dotProduct(line, startToPoint) / lengthSquared;
}

double Math::getDistanceToLineSegment(const Point& lStart, const Point& lEnd, const Point& point) {
	Point line = lEnd - lStart;
	Point startToPoint = point - lStart;
	double lengthSquared = getDistanceSquared(lStart, lEnd);
	double t = dotProduct(line, startToPoint) / lengthSquared;

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
	double cP = crossProduct(nEnd, nPoint);

	if(cP > 0)
		return 1;
	
	if(cP < 0)
		return -1;
	
	return 0;
}

double Math::getDistanceToLine(const Point& lStart, const Point& lEnd, const Point& point) {
	Point line = lEnd - lStart;
	Point startToPoint = point - lStart;
	double lengthSquared = getDistanceSquared(lStart, lEnd);
	double t = dotProduct(line, startToPoint) / lengthSquared;


	return getDistance(point, lStart + t * line);
}

double Math::getAngleBetweenVectors(const Point& v1, const Point& v2) {
	return std::acos(dotProduct(v1, v2) / (getLength(v1) * getLength(v2)));
}

Point Math::getVectorFromOrientation(double o) {
	return Point(std::cos(o), std::sin(o));
}

double Math::getOrientationFromVector(const Point& v) {
	return std::atan2(v.y, v.x);
}

double Math::getAngleMedian(double a1, double a2) {
	double diff = a2 - a1;

	if(diff > 180) {
		diff = diff - 360;
	} else if(diff < -180){
		diff = diff + 360;
	}

	return a1 + diff/2.f;
}

double Math::lerp(double start, double end, double alpha) {
	return start + (end-start) * alpha;
}

double Math::clamp(double value, double min, double max) {
	return std::max(min, std::min(max, value));
}

double Math::getAngleDifferenceInDegree(double source, double target) {
	double angle = target - source;

	if(angle > 180) {
		angle = angle - 360;
	} else if(angle < -180){
		angle = angle + 360;
	}

	return angle;
}

double Math::getAngleDifferenceInRad(double source, double target) {
	double angle = target - source;

	if(angle > M_PI) {
		angle = angle - 2 * M_PI;
	} else if(angle < -M_PI){
		angle = angle + 2 * M_PI;
	}

	return angle;
}

double Math::normalizeRad(double angle) {
	if(angle > M_PI) {
		angle = angle - 2 * M_PI;
	} else if(angle < -M_PI){
		angle = angle + 2 * M_PI;
	}

	return angle;
}

double Math::normalizeDegree(double angle) {
	if(angle > 180) {
		angle = angle - 360;
	} else if(angle < -180){
		angle = angle + 360;
	}

	return angle;
}

bool Math::doesLineSegmentIntersectAxisAlignedRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle) {
	// Find min and max X for the segment
	double minX = lStart.x;
	double maxX = lEnd.x;

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
	double minY = lStart.y;
	double maxY = lEnd.y;

	double dx = lEnd.x - lStart.x;

	if(std::abs(dx) > EPS) {
		double a = (lEnd.y - lStart.y) / dx;
		double b = lStart.y - a * lStart.x;
		minY = a * minX + b;
		maxY = a * maxX + b;
	}

	if(minY > maxY) {
		double tmp = maxY;
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
}

bool Math::doesLineSegmentIntersectNonAxisAlignedRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle) {
	const Point* rect = rectangle.getPointsInflated();

	return (doLineSegmentsIntersect(lStart, lEnd, rect[0], rect[1]) ||
	        doLineSegmentsIntersect(lStart, lEnd, rect[1], rect[2]) ||
	        doLineSegmentsIntersect(lStart, lEnd, rect[2], rect[3]) ||
	        doLineSegmentsIntersect(lStart, lEnd, rect[3], rect[0]) ||
	        isPointInRectangle(lStart, rectangle) ||
	        isPointInRectangle(lEnd, rectangle));
}

bool Math::isPointInRectangle(const Point& p, const Rectangle& rectangle) {
	// https://math.stackexchange.com/a/190373
	// (0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
	bool isInAxisAligned = isPointInAxisAlignedRectangle(p, rectangle);

	if(rectangle.getIsAxisAligned()) {
		return isInAxisAligned;
	} else {
		if(!isInAxisAligned) {
			return false;
		}

		const Point* rect = rectangle.getPointsInflated();

		Point ap = rect[0] - p;
		Point ab = rect[0] - rect[1];
		Point ad = rect[0] - rect[3];

		return (0 < dotProduct(ap, ab) && dotProduct(ap, ab) < dotProduct(ab, ab)) && (0 < dotProduct(ap, ad) && dotProduct(ap, ad) < dotProduct(ad, ad));
	}
}

bool Math::doesLineSegmentIntersectNonInflatedRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle) {
	const Point* rect = rectangle.getPointsNonInflated();

	return (doLineSegmentsIntersect(lStart, lEnd, rect[0], rect[1]) ||
	        doLineSegmentsIntersect(lStart, lEnd, rect[1], rect[2]) ||
	        doLineSegmentsIntersect(lStart, lEnd, rect[2], rect[3]) ||
	        doLineSegmentsIntersect(lStart, lEnd, rect[3], rect[0]) ||
	        isPointInNonInflatedRectangle(lStart, rectangle) ||
	        isPointInNonInflatedRectangle(lEnd, rectangle));
}

bool Math::isPointInNonInflatedRectangle(const Point& p, const Rectangle& rectangle) {
	const Point* rect = rectangle.getPointsNonInflated();

	Point ap = rect[0] - p;
	Point ab = rect[0] - rect[1];
	Point ad = rect[0] - rect[3];

	return (0 < dotProduct(ap, ab) && dotProduct(ap, ab) < dotProduct(ab, ab)) && (0 < dotProduct(ap, ad) && dotProduct(ap, ad) < dotProduct(ad, ad));
}

bool Math::isPointInAxisAlignedRectangle(const Point& p, const Rectangle& rectangle) {
	return !(p.x < rectangle.getMinXInflated() || p.x > rectangle.getMaxXInflated() ||
	         p.y < rectangle.getMinYInflated() || p.y > rectangle.getMaxYInflated());
}

bool Math::doesLineSegmentIntersectRectangle(const Point& lStart, const Point& lEnd, const Rectangle& rectangle) {
	if(rectangle.getIsAxisAligned()) {
		return doesLineSegmentIntersectAxisAlignedRectangle(lStart, lEnd, rectangle);
	} else {
		return doesLineSegmentIntersectNonAxisAlignedRectangle(lStart, lEnd, rectangle);
	}
}
