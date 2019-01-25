#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include <visualization_msgs/Marker.h>
#include "agent/path_planning/Point.h"
#include "agent/path_planning/Rectangle.h"
#include "RobotHardwareProfile.h"

class Path {
public:
	float maxReservationLength = 20;
	
private:
	float startTimeOffset = 0;
	std::vector<Point> nodes;
	std::vector<float> waitTimes;
	RobotHardwareProfile* hardwareProfile;

	// Computed internally for motion planner
	std::vector<float> departureTimes;
	float distance = 0;
	float duration = 0;
	float batteryConsumption = 0;

public:
	explicit Path() = default;
	explicit Path(float startTimeOffset, std::vector<Point> nodes, std::vector<float> waitTimes, RobotHardwareProfile* hardwareProfile);
	
	virtual ~Path() = default;

	const std::vector<Point>& getNodes() const;
	const std::vector<float>& getWaitTimes() const;
	const std::vector<float>& getDepartureTimes() const;

	float getDistance() const;
	float getDuration() const;
	float getBatteryConsumption() const;
	float getStartTimeOffset() const;
	RobotHardwareProfile* getRobotHardwareProfile() const;

	const std::vector<Rectangle> generateReservations() const;
	
	// ROS visualisation
	visualization_msgs::Marker getVisualizationMsgPoints();
	visualization_msgs::Marker getVisualizationMsgLines();

};

#endif /* AGENT_PATH_H_ */