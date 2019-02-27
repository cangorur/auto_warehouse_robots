#ifndef PROTOTYPE_RECTANGLE_HPP
#define PROTOTYPE_RECTANGLE_HPP

#include "agent/path_planning/Point.h"

/* Class representing an obstacle or a reservation (which is a timed obstacle). A rotated rectangle is used as geometric representation */
class Rectangle {
private:
	// Center position of the rectangle
	Point pos;
	
	// Size of the rectangle
	Point size;
	
	// Rotation in degree
	float rotation;

	// Start/End time if the reservation is a timed one
	double startTime;
	double endTime;
	
	// The owner id of the reservation
	int ownerId;
	
	// ==== Internal data for faster physic processing
	// Corner points of the inflated rectangle
	Point pointsInflated[4];

	// Corner points of the non-inflated rectangle
	Point pointsNonInflated[4];
	
	// Is this rectangle axis aligned (rotation is multiple of 90°)
	bool isAxisAligned;
	
	// Min/Max extension of the AABB covering this rectangle
	double minXInflated, maxXInflated, minYInflated, maxYInflated;

public:
	Rectangle(Point pos, Point size, float rotation);
	Rectangle(Point pos, Point size, float rotation, double startTime, double endTime, int ownerId);
	
	// Getter
	const Point* getPointsInflated() const;
	const Point* getPointsNonInflated() const;
	Point getPosition() const;
	Point getSize() const;
	float getRotation() const;
	int getOwnerId() const;
	
	bool getIsAxisAligned() const;
	double getMinXInflated() const;
	double getMaxXInflated() const;
	double getMinYInflated() const;
	double getMaxYInflated() const;

	double getStartTime() const;
	double getEndTime() const;
	double getFreeAfter() const;
	
	/** Checks whether this rectangle overlaps with the specified time range (= time based "collision")
	 * @param start Start time of the time range
	 * @param end End time of the time range
	 * @param ownerId Owner Id of the caller. This is used to check if the reservation belongs to the owner
	 * @return True iff the time ranges overlap AND the rectangle does not belong the the specified owner id */
	bool doesOverlapTimeRange(double start, double end, int ownerId) const;
};

// Overloaded operators for rectangle comparison
bool operator ==(const Rectangle& left, const Rectangle& right);
bool operator !=(const Rectangle& left, const Rectangle& right);


#endif //PROTOTYPE_RECTANGLE_HPP
