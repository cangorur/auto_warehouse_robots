#include <algorithm>
#include <include/agent/path_planning/Rectangle.h>

#include "agent/path_planning/Rectangle.h"
#include "Math.h"

Rectangle::Rectangle(Point pos_, Point size_, float rotation_, double startTime, double endTime, int ownerId) :
		pos(pos_),
		size(size_),
		rotation(rotation_),
		startTime(startTime),
		endTime(endTime),
		ownerId(ownerId)
{
	isAxisAligned = false;
	if(static_cast<int>(std::roundf(rotation)) % 90 == 0) {
		rotation = std::roundf(rotation);
		isAxisAligned = true;
	}
	
	// Todo change color
	if(startTime == endTime && endTime == -1) {
		//color = sf::Color(0, 0, 200);
	} else {
		//color = sf::Color(200, 0, 200);
	}

	// Generate Points
	Point diagonal = size * 0.5f;
	Point diagonalMirrored = Point(diagonal.x, -diagonal.y);
	pointsNonInflated[0] = pos + Math::rotateVector(diagonal, rotation);
	pointsNonInflated[2] = pos + Math::rotateVector(diagonal, rotation + 180);
	pointsNonInflated[1] = pos + Math::rotateVector(diagonalMirrored, rotation);
	pointsNonInflated[3] = pos + Math::rotateVector(diagonalMirrored, rotation + 180);
	
	// Generate inflated points
	Point sizeInflated = Point(size.x + ROBOT_RADIUS * 2, size.y + ROBOT_RADIUS * 2);
	Point diagonalInflated = Point(size.x + ROBOT_RADIUS * 2, size.y + ROBOT_RADIUS * 2) * 0.5f;
	Point diagonalInflatedMirrored = Point(diagonalInflated.x, -diagonalInflated.y);
	
	pointsInflated[0] = pos + Math::rotateVector(diagonalInflated, rotation);
	pointsInflated[2] = pos + Math::rotateVector(diagonalInflated, rotation + 180);
	pointsInflated[1] = pos + Math::rotateVector(diagonalInflatedMirrored, rotation);
	pointsInflated[3] = pos + Math::rotateVector(diagonalInflatedMirrored, rotation + 180);
	
	minXInflated = std::min({pointsInflated[0].x, pointsInflated[1].x, pointsInflated[2].x, pointsInflated[3].x});
	maxXInflated = std::max({pointsInflated[0].x, pointsInflated[1].x, pointsInflated[2].x, pointsInflated[3].x});
	minYInflated = std::min({pointsInflated[0].y, pointsInflated[1].y, pointsInflated[2].y, pointsInflated[3].y});
	maxYInflated = std::max({pointsInflated[0].y, pointsInflated[1].y, pointsInflated[2].y, pointsInflated[3].y});
}

Rectangle::Rectangle(Point pos, Point size, float rotation) :
		Rectangle(pos, size, rotation, -1, -1, -1) {}

const Point* Rectangle::getPointsInflated() const {
	return pointsInflated;
}

Point Rectangle::getPosition() const {
	return pos;
}

Point Rectangle::getSize() const {
	return size;
}

double Rectangle::getMinXInflated() const {
	return minXInflated;
}

double Rectangle::getMaxXInflated() const {
	return maxXInflated;
}

double Rectangle::getMinYInflated() const {
	return minYInflated;
}

double Rectangle::getMaxYInflated() const {
	return maxYInflated;
}

bool Rectangle::getIsAxisAligned() const {
	return isAxisAligned;
}

float Rectangle::getRotation() const {
	return rotation;
}

bool Rectangle::doesOverlapTimeRange(double start, double end, int ownerId) const {
	return (start <= this->endTime) && (end >= this->startTime) && ownerId != this->ownerId;
}

double Rectangle::getFreeAfter() const {
	return endTime;
}

double Rectangle::getStartTime() const {
	return startTime;
}

double Rectangle::getEndTime() const {
	return endTime;
}

int Rectangle::getOwnerId() const {
	return ownerId;
}

const Point* Rectangle::getPointsNonInflated() const {
	return pointsNonInflated;
}

bool operator ==(const Rectangle& left, const Rectangle& right) {
	return left.getStartTime() == right.getStartTime() &&
	       left.getEndTime() == right.getEndTime() &&
	       left.getPosition() == right.getPosition() &&
	       left.getSize() == right.getSize() &&
	       left.getRotation() == right.getRotation() &&
	       left.getOwnerId() == right.getOwnerId();			
}

bool operator !=(const Rectangle& left, const Rectangle& right) {
	return !(left == right);
}

