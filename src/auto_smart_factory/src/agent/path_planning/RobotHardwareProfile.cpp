#include <cmath>
#include "agent/path_planning/RobotHardwareProfile.h"

RobotHardwareProfile::RobotHardwareProfile(float maxDrivingSpeed, float maxTurningSpeed, float idleBatteryConsumption, float drivingBatteryConsumption) :
		maxDrivingSpeed(maxDrivingSpeed),
		maxTurningSpeed(maxTurningSpeed),
		idleBatteryConsumption(idleBatteryConsumption),
		drivingBatteryConsumption(drivingBatteryConsumption)
{
}

float RobotHardwareProfile::getIdleBatteryConsumption(double time) const {
	return static_cast<float>(idleBatteryConsumption * time);
}

// Todo include idle consumption?
float RobotHardwareProfile::getDrivingBatteryConsumption(double time) const {
	return static_cast<float>(drivingBatteryConsumption * time + getIdleBatteryConsumption(time));
}

double RobotHardwareProfile::getDrivingDuration(float distance) const {
	return distance/(maxDrivingSpeed * averageDrivingEfficiency);
}

double RobotHardwareProfile::getTurningDuration(float angle) const {
	return std::fabs(angle)/(maxTurningSpeed * averageTurningEfficiency);
}

