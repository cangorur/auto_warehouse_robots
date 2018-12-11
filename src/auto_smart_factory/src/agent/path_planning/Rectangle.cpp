#include <algorithm>

#include "agent/path_planning/Rectangle.h"
#include "Math.h"

Rectangle::Rectangle(Point pos_, Point size_, float rotation_) :
		pos(pos_),
		size(size_),
		rotation(rotation_) {
	
	if(static_cast<int>(std::roundf(rotation)) % 90 == 0) {
		rotation = std::roundf(rotation);
		isAxisAligned = true;
	}
	
	Point sizeInflated = Point(size.x + ROBOT_DIAMETER * 2, size.y + ROBOT_DIAMETER * 2);

	/*
	rectangleShape = sf::RectangleShape(Point(size.x, size.y));
	rectangleShape.setOrigin(size * 0.5f);
	rectangleShape.setPosition(pos);
	rectangleShape.setRotation(rotation);
	rectangleShape.setFillColor(color);

	rectangleShapeInflated = sf::RectangleShape(sizeInflated);
	rectangleShapeInflated.setOrigin(sizeInflated * 0.5f);
	rectangleShapeInflated.setPosition(pos);
	rectangleShapeInflated.setRotation(rotation);
	rectangleShapeInflated.setFillColor(sf::Color(color.r, color.g, color.b, 80));*/
	
	// Generate inflated points
	Point diagonalInflated = Point(size.x + ROBOT_DIAMETER * 2, size.y + ROBOT_DIAMETER * 2) * 0.5f;
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

/*void Rectangle::draw(sf::RenderWindow& renderWindow) {
	renderWindow.draw(rectangleShapeInflated);
	renderWindow.draw(rectangleShape);
}*/

bool Rectangle::isInsideInflated(const Point& point) const {
	bool isInAxisAligned = isInsideAxisAlignedInflated(point);
	
	if(isAxisAligned) {
		 return isInAxisAligned;
	} else {
		if(!isInAxisAligned) {
			return false;
		}
		
		Point ab = pointsInflated[1] - pointsInflated[0];
		Point bc = pointsInflated[2] - pointsInflated[1];
		Point ap = point - pointsInflated[0];
		Point bp = point - pointsInflated[1];

		return (0 <= Math::dotProduct(ab, ap) && Math::dotProduct(ab, ap) <= Math::dotProduct(ab, ab) &&
		        0 <= Math::dotProduct(bc, bp) && Math::dotProduct(bc, bp) <= Math::dotProduct(bc, bc));	
	}
}

bool Rectangle::isInsideAxisAlignedInflated(const Point& point) const {
	return !(point.x < minXInflated || point.x > maxXInflated ||
	         point.y < minYInflated || point.y > maxYInflated);
}

Point* Rectangle::getPointsInflated() {
	return pointsInflated;
}

Point Rectangle::getPosition() const {
	return pos;
}

Point Rectangle::getSize() const {
	return size;
}

float Rectangle::getMinXInflated() const {
	return minXInflated;
}

float Rectangle::getMaxXInflated() const {
	return maxXInflated;
}

float Rectangle::getMinYInflated() const {
	return minYInflated;
}

float Rectangle::getMaxYInflated() const {
	return maxYInflated;
}

bool Rectangle::getIsAxisAligned() const {
	return isAxisAligned;
}

