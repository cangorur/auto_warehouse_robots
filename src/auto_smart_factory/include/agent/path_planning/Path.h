#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include <visualization_msgs/Marker.h>
#include "agent/path_planning/Point.h"
#include "agent/path_planning/Rectangle.h"
#include "RobotHardwareProfile.h"

class Path {
public:
	double maxReservationLength = 20;
	
private:
	double startTimeOffset = 0;
	std::vector<Point> nodes;
	std::vector<double> waitTimes;
	RobotHardwareProfile* hardwareProfile;

	// Computed internally for motion planner
	std::vector<double> departureTimes;
	float distance = 0;
	float duration = 0;
	float batteryConsumption = 0;

public:
	explicit Path() = default;
	explicit Path(double startTimeOffset, std::vector<Point> nodes, std::vector<double> waitTimes, RobotHardwareProfile* hardwareProfile);
	
	virtual ~Path() = default;

	const std::vector<Point>& getNodes() const;
	const std::vector<double>& getWaitTimes() const;
	const std::vector<double>& getDepartureTimes() const;

	float getDistance() const;
	double getDuration() const;
	float getBatteryConsumption() const;
	double getStartTimeOffset() const;
	RobotHardwareProfile* getRobotHardwareProfile() const;

	const std::vector<Rectangle> generateReservations(int ownerId) const;
	
	// ROS visualisation
	visualization_msgs::Marker getVisualizationMsgPoints();
	visualization_msgs::Marker getVisualizationMsgLines();

};

#endif /* AGENT_PATH_H_ */