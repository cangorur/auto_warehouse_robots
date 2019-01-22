#ifndef PROTOTYPE_MAP_H
#define PROTOTYPE_MAP_H

#include <vector>

#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/RobotHardwareProfile.h"
#include "agent/path_planning/TimedLineOfSightResult.h"
#include "auto_smart_factory/Tray.h"
#include <auto_smart_factory/WarehouseConfiguration.h>

class Map {
private:
	float width;
	float height;
	float margin;
	auto_smart_factory::WarehouseConfiguration warehouseConfig;
	
	std::vector<Rectangle> obstacles;
	std::vector<Rectangle> reservations;
	ThetaStarMap thetaStarMap;

public:
	Map() = default;
	Map(auto_smart_factory::WarehouseConfiguration warehouseConfig, std::vector<Rectangle> &obstacles);

	visualization_msgs::Marker getVisualization();
	
	bool isInsideAnyInflatedObstacle(const Point& point) const;	
	bool isStaticLineOfSightFree(const Point& pos1, const Point& pos2) const;
	bool isTimedLineOfSightFree(const Point& pos1, float startTime, const Point& pos2, float endTime) const;
	TimedLineOfSightResult whenIsTimedLineOfSightFree(const Point& pos1, float startTime, const Point& pos2, float endTime) const;

	// Does not check against static obstacles, this is only used to verify a already planned connection
	bool isTimedConnectionFree(const Point& pos1, const Point& pos2, float startTime, float waitingTime, float drivingTime) const;
	
	Path getThetaStarPath(const Point& start, const Point& end, float startingTime, RobotHardwareProfile* hardwareProfile);
	
	Point getRandomFreePoint() const;

	void deleteReservations();
	void addReservations(std::vector<Rectangle> newReservations);
	
	// Getter
	float getWidth() const;
	float getHeight() const;
	float getMargin() const;
	
	bool isPointInMap(const Point& pos) const;

};


#endif //PROTOTYPE_MAP_H
