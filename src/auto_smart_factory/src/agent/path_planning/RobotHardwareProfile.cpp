#include <cmath>
#include <include/agent/path_planning/RobotHardwareProfile.h>
#include <algorithm>

#include "agent/path_planning/RobotHardwareProfile.h"

RobotHardwareProfile::RobotHardwareProfile(double maxDrivingSpeed, double maxTurningSpeed, double idleBatteryConsumption, double drivingBatteryConsumption) :
		maxDrivingSpeed(maxDrivingSpeed),
		maxTurningSpeed(maxTurningSpeed),
		idleBatteryConsumption(idleBatteryConsumption),
		drivingBatteryConsumption(drivingBatteryConsumption)
{
}

double RobotHardwareProfile::getIdleBatteryConsumption(double time) const {
	return idleBatteryConsumption * time;
}

double RobotHardwareProfile::getDrivingBatteryConsumption(double distance) const {
	return drivingBatteryConsumption * distance + getIdleBatteryConsumption(getDrivingDuration(distance));
}

double RobotHardwareProfile::getDrivingDuration(double distance) const {
	return distance / (maxDrivingSpeed * averageDrivingEfficiency);
}

double RobotHardwareProfile::getTurningDuration(double angle) const {
	// In degree
	angle = std::abs(angle);
	if(angle < onSpotTurningAngle) {
		return angle / (drivingTurningEfficiency * maxTurningSpeed);
	} else {
		return getOnSpotTurningTime(angle);	
	}
}

double RobotHardwareProfile::getTimeUncertaintyPercentage() const {
	return timeUncertaintyPercentage;
}

double RobotHardwareProfile::getTimeUncertaintyAbsolute() const {
	return timeUncertaintyAbsolute;
}

bool RobotHardwareProfile::performsOnSpotTurn(double angleInDeg, bool firstPoint) const {
	if(firstPoint) {
		return angleInDeg >= onSpotTurningAngleFirstPoint;
	} else {
		return angleInDeg >= onSpotTurningAngle;	
	}
}

double RobotHardwareProfile::getOnSpotTurningTime(double angle) const {
	// Speed equals angle at this speed
	double time = 0;
	
	// Min turning segment
	double angleAtMinSpeed = std::min(minTurningSpeed, angle);
	angle -= angleAtMinSpeed;
	time += angleAtMinSpeed / minTurningSpeed;
	
	// Linear decelerating segment
	double angleAtLinearDeceleration = std::min(maxTurningSpeed - minTurningSpeed, angle);
	angle -= angleAtLinearDeceleration;
	double averageSpeedDuringDeceleration = minTurningSpeed + (angleAtLinearDeceleration / 2.f);
	time += angleAtLinearDeceleration / averageSpeedDuringDeceleration;

	// Max turning speed
	double angleAtMaxSpeed = angle;
	time += angleAtMaxSpeed / maxTurningSpeed;
	
	return time;
}



