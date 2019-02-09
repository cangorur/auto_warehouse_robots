#include <cmath>
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

// Todo include idle consumption?
double RobotHardwareProfile::getDrivingBatteryConsumption(double time) const {
	return drivingBatteryConsumption * time + getIdleBatteryConsumption(time);
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

