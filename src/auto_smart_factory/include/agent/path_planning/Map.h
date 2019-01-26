#ifndef PROTOTYPE_MAP_H
#define PROTOTYPE_MAP_H

#include <vector>

#include "auto_smart_factory/Tray.h"
#include <auto_smart_factory/WarehouseConfiguration.h>
#include "agent/path_planning/Rectangle.h"
#include "agent/path_planning/GridNode.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/OrientedPoint.h"
#include "agent/path_planning/RobotHardwareProfile.h"
#include "agent/path_planning/TimedLineOfSightResult.h"

class Map {
	static int visualisationId;
	
private:
	float width;
	float height;
	float margin;
	auto_smart_factory::WarehouseConfiguration warehouseConfig;
	
	std::vector<Rectangle> obstacles;
	std::vector<Rectangle> reservations;
	ThetaStarMap thetaStarMap;
	RobotHardwareProfile* hardwareProfile;

public:
	Map() = default;
	Map(auto_smart_factory::WarehouseConfiguration warehouseConfig, std::vector<Rectangle> &obstacles, RobotHardwareProfile* hardwareProfile);

	visualization_msgs::Marker getObstacleVisualization();
	visualization_msgs::Marker getReservationVisualization();
	
	bool isInsideAnyInflatedObstacle(const Point& point) const;	
	bool isStaticLineOfSightFree(const Point& pos1, const Point& pos2) const;
	bool isTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime) const;
	TimedLineOfSightResult whenIsTimedLineOfSightFree(const Point& pos1, double startTime, const Point& pos2, double endTime) const;

	// Does not check against static obstacles, this is only used to verify a already planned connection
	bool isTimedConnectionFree(const Point& pos1, const Point& pos2, double startTime, float waitingTime, float drivingTime) const;
	
	Point getRandomFreePoint() const;

	void deleteExpiredReservations(double time);
	void addReservations(std::vector<Rectangle> newReservations);
		
	Path getThetaStarPath(const Point& start, const Point& end, double startingTime);
	Path getThetaStarPath(const Point& start, const auto_smart_factory::Tray& end, double startingTime);
	Path getThetaStarPath(const auto_smart_factory::Tray& start, const Point& end, double startingTime);
	Path getThetaStarPath(const auto_smart_factory::Tray& start, const auto_smart_factory::Tray& end, double startingTime);
	
	// Getter
	float getWidth() const;
	float getHeight() const;
	float getMargin() const;
	
	bool isPointInMap(const Point& pos) const;
	
	OrientedPoint getPointInFrontOfTray(const auto_smart_factory::Tray& tray);
};


#endif //PROTOTYPE_MAP_H
