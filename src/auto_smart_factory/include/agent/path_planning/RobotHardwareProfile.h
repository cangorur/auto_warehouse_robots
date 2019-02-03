#ifndef PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
#define PROTOTYPE_ROBOTHARDWAREPROFILE_HPP

class RobotHardwareProfile {
private:
	double maxDrivingSpeed; // Length-Unit / Time-Unit
	double maxTurningSpeed; // Angle / Time-Unit
	double idleBatteryConsumption;
	double drivingBatteryConsumption;

	// Used to get average estimates from max speed values 
	const double averageDrivingEfficiency = 0.75f;
	const double averageTurningEfficiency = 0.45f;

public:
	RobotHardwareProfile(double maxDrivingSpeed, double maxTurningSpeed, double idleBatteryConsumption, double drivingBatteryConsumption);

	double getIdleBatteryConsumption(double time) const;
	double getDrivingBatteryConsumption(double time) const;
	double getDrivingDuration(double distance) const;
	double getTurningDuration(double angle) const;
};


#endif //PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
