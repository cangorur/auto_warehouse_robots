#ifndef PROTOTYPE_RECTANGLE_HPP
#define PROTOTYPE_RECTANGLE_HPP

#include "agent/path_planning/Point.h"

// Defined as 0.25 + error margin
#define ROBOT_DIAMETER 0.3

class Rectangle {
private:	
	Point pos;
	Point size;
	float rotation;
	
	//sf::RectangleShape rectangleShape;
	//sf::RectangleShape rectangleShapeInflated;
	
	// Faster physic processing
	Point pointsInflated[4];
	bool isAxisAligned;
	float minXInflated, maxXInflated, minYInflated, maxYInflated;

public:
	Rectangle(Point pos, Point size, float rotation);
	
	//void draw(sf::RenderWindow& renderWindow) override;
	
	bool isInsideInflated(const Point& point) const;
	Point* getPointsInflated();
	Point getPosition() const;
	Point getSize() const;
	
	bool getIsAxisAligned() const;
	float getMinXInflated() const;
	float getMaxXInflated() const;
	float getMinYInflated() const;
	float getMaxYInflated() const;

private:
	bool isInsideAxisAlignedInflated(const Point& point) const;	

};


#endif //PROTOTYPE_RECTANGLE_HPP
