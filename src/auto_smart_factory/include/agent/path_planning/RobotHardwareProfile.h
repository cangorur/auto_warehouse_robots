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

	const double onSpotTurningAngle = 55.f;
	const double drivingTurningEfficiency = 0.5f;
	const double onSpotTurningEfficiency = 0.3f;

	// For reservations
	const double timeUncertaintyPercentage = 0.15f;
	const double timeUncertaintyAbsolute = 0.6f;

public:
	RobotHardwareProfile(double maxDrivingSpeed, double maxTurningSpeed, double idleBatteryConsumption, double drivingBatteryConsumption);

	double getIdleBatteryConsumption(double time) const;
	double getDrivingBatteryConsumption(double time, double distance) const;
	double getDrivingDuration(double distance) const;
	double getTurningDuration(double angle) const;
	
	double getTimeUncertaintyPercentage() const;
	double getTimeUncertaintyAbsolute() const;
	double getOnSpotTurningAngle() const;
};


#endif //PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
