#ifndef PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
#define PROTOTYPE_ROBOTHARDWAREPROFILE_HPP

class RobotHardwareProfile {
private:
	float maxDrivingSpeed; // Length-Unit / Time-Unit
	float maxTurningSpeed; // Angle / Time-Unit
	float idleBatteryConsumption;
	float drivingBatteryConsumption;

	// Used to get average estimates from max speed values 
	const float averageDrivingEfficiency = 0.9f;
	const float averageTurningEfficiency = 0.9f;

public:
	RobotHardwareProfile(float maxDrivingSpeed, float maxTurningSpeed, float idleBatteryConsumption, float drivingBatteryConsumption);

	float getIdleBatteryConsumption(float time) const;
	float getDrivingBatteryConsumption(float time) const;
	float getDrivingDuration(float distance) const;
	float getTurningDuration(float angle) const;
};


#endif //PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
