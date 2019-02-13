#ifndef PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
#define PROTOTYPE_ROBOTHARDWAREPROFILE_HPP

class RobotHardwareProfile {
private:
	double maxDrivingSpeed; // Length-Unit / Time-Unit
	double maxTurningSpeed; // Angle (deg) / Time-Unit
	double minTurningSpeed = 17.0;
	double idleBatteryConsumption;
	double drivingBatteryConsumption;

	// Used to get average estimates from max speed values 
	const double averageDrivingEfficiency = 0.82f;

	const double onSpotTurningAngle = 55.f;
	const double onSpotTurningAngleFirstPoint = 15.f;
	const double drivingTurningEfficiency = 0.65f;

	// For reservations == not used
	const double timeUncertaintyPercentage = 0.075f;
	const double timeUncertaintyAbsolute = 0.08f;

public:
	RobotHardwareProfile(double maxDrivingSpeed, double maxTurningSpeed, double idleBatteryConsumption, double drivingBatteryConsumption);

	double getIdleBatteryConsumption(double time) const;
	double getDrivingBatteryConsumption(double distance) const;
	double getDrivingDuration(double distance) const;
	double getTurningDuration(double angle) const;
	
	double getTimeUncertaintyPercentage() const;
	double getTimeUncertaintyAbsolute() const;
	bool performsOnSpotTurn(double angleInDeg, bool firstPoint) const;

private:
	double getOnSpotTurningTime(double angle) const;
};


#endif //PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
