#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include "visualization_msgs/Marker.h"
#include "agent/path_planning/Point.h"
#include "agent/path_planning/Rectangle.h"
#include "RobotHardwareProfile.h"
#include "TimingCalculator.h"

class Path {
public:
	double maxReservationDistance = 1.75f;
	double reservationTimeMarginAhead = 0.f;
	double reservationTimeMarginBehind = 0.f;
	
private:
	double startTimeOffset = 0;
	std::vector<Point> nodes;
	std::vector<double> waitTimes;
	RobotHardwareProfile* hardwareProfile;
	
	OrientedPoint start;
	OrientedPoint end;
	bool isValidPath;

	// Computed internally for motion planner
	std::vector<double> departureTimes;
	double distance;
	double duration;
	double batteryConsumption;
	
	// For timing
	TimingCalculator timing;
	double targetReservationTime;
	std::vector<double> drivingTimes;
	std::vector<double> onSpotTimes;

public:
	explicit Path();
	explicit Path(double startTimeOffset, std::vector<Point> nodes, std::vector<double> waitTimes, RobotHardwareProfile* hardwareProfile, double targetReservationTime, OrientedPoint start, OrientedPoint end);
	
	virtual ~Path() = default;

	const std::vector<Point>& getNodes() const;
	const std::vector<double>& getWaitTimes() const;
	const std::vector<double>& getDepartureTimes() const;

	double getDistance() const;
	double getDuration() const;
	double getBatteryConsumption() const;
	double getStartTimeOffset() const;
	RobotHardwareProfile* getRobotHardwareProfile() const;

	const std::vector<Rectangle> generateReservations(int ownerId) const;
	
	// ROS visualisation
	visualization_msgs::Marker getVisualizationMsgLines(std_msgs::ColorRGBA color);

	OrientedPoint getStart();
	OrientedPoint getEnd();
	bool isValid() const;
};

#endif /* AGENT_PATH_H_ */