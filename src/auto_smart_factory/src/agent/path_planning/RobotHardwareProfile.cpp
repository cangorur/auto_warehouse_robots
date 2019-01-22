#include <cmath>
#include "agent/path_planning/RobotHardwareProfile.h"

RobotHardwareProfile::RobotHardwareProfile(float maxDrivingSpeed, float maxTurningSpeed, float idleBatteryConsumption, float drivingBatteryConsumption) :
		maxDrivingSpeed(maxDrivingSpeed),
		maxTurningSpeed(maxTurningSpeed),
		idleBatteryConsumption(idleBatteryConsumption),
		drivingBatteryConsumption(drivingBatteryConsumption)
{
}

float RobotHardwareProfile::getIdleBatteryConsumption(float time) const {
	return idleBatteryConsumption * time;
}

// Todo include idle consumption?
float RobotHardwareProfile::getDrivingBatteryConsumption(float time) const {
	return drivingBatteryConsumption * time + getIdleBatteryConsumption(time);
}

float RobotHardwareProfile::getDrivingDuration(float distance) const {
	return distance/(maxDrivingSpeed * averageDrivingEfficiency);
}

float RobotHardwareProfile::getTurningDuration(float angle) const {
	return std::fabs(angle)/(maxTurningSpeed * averageTurningEfficiency);
}

