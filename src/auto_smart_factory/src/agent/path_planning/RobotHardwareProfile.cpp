#include <cmath>
#include <include/agent/path_planning/RobotHardwareProfile.h>

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

double RobotHardwareProfile::getDrivingBatteryConsumption(double time, double distance) const {
	return drivingBatteryConsumption * distance + getIdleBatteryConsumption(time);
}

double RobotHardwareProfile::getDrivingDuration(double distance) const {
	return distance / (maxDrivingSpeed * averageDrivingEfficiency);
}

double RobotHardwareProfile::getTurningDuration(double angle) const {
	angle = std::abs(angle);
	if(angle < onSpotTurningAngle) {
		return angle / (drivingTurningEfficiency * maxTurningSpeed);
	} else {
		return angle / (onSpotTurningEfficiency * maxTurningSpeed);	
	}
}

double RobotHardwareProfile::getTimeUncertaintyPercentage() const {
	return timeUncertaintyPercentage;
}

double RobotHardwareProfile::getTimeUncertaintyAbsolute() const {
	return timeUncertaintyAbsolute;
}

double RobotHardwareProfile::getOnSpotTurningAngle() const {
	return onSpotTurningAngle;
}

