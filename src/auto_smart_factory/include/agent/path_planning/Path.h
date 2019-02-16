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
	double maxDrivingReservationDuration = 1.75f;
	double reservationTimeMarginAhead = 0.15f;
	double reservationTimeMarginBehind = 0.1f;
	
	double finalPointAdditionalTime = 0.35f;
	
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

	OrientedPoint getStart() const;
	OrientedPoint getEnd() const;
	bool isValid() const;
	
	static double getReservationSize();

private:
	void generateReservationsForSegment(std::vector<Rectangle>& reservations, Point startPoint, Point endPoint, double timeAtStartPoint, double deltaDuration, int ownerId) const;
	void generateReservationsForCurvePoints(std::vector<Rectangle>& reservations, std::vector<Point> points, double timeAtStartPoint, double deltaTime, int ownerId) const;
	void generateReservationForTray(std::vector<Rectangle>& reservations, double timeAtStartPoint, int ownerId) const;
};

#endif /* AGENT_PATH_H_ */