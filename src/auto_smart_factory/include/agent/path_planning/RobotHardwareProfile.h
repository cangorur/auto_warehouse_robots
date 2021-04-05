#ifndef PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
#define PROTOTYPE_ROBOTHARDWAREPROFILE_HPP

// Class which contains robot specific values to estimate the robot behaviour. This includes driving and turning times and battery consumption
class RobotHardwareProfile {
private:
	// Maximum driving speed of this robot
	double maxDrivingSpeed; // Length-Unit / Time-Unit
	
	// Maximum turning speed
	double maxTurningSpeed; // Angle (deg) / Time-Unit
	
	// Minimum turning speed
	double minTurningSpeed = 17.0;
	
	// Battery consumption per second
	double idleBatteryConsumption;
	
	// Battery consumption per distance unit 
	double drivingBatteryConsumption;

	// Used to get average estimates from max speed values 
	const double averageDrivingEfficiency = 0.82f;

	// Angle above which a rotation counts as on Spot turn
	const double onSpotTurningAngle = 55.f;

	// Angle above which a rotation counts as on Spot turn for the initial path rotation
	const double onSpotTurningAngleFirstPoint = 15.f;
	
	// Efficiency for turning while driving
	const double drivingTurningEfficiency = 0.65f;

	// Uncertainty used during path calculation and reservation generation. Describes how much leeway is necessary for a valid path 
	const double timeUncertaintyPercentage = 0.075f;
	const double timeUncertaintyAbsolute = 0.08f;

public:
	// Construct RobotHardwareProfile from robot config data
	RobotHardwareProfile(double maxDrivingSpeed, double maxTurningSpeed, double idleBatteryConsumption, double drivingBatteryConsumption);

	// Getter
	double getIdleBatteryConsumption(double time) const;
	double getDrivingBatteryConsumption(double distance) const;
	double getDrivingDuration(double distance) const;
	double getTurningDuration(double angle) const;
	
	double getTimeUncertaintyPercentage() const;
	double getTimeUncertaintyAbsolute() const;
	
	/** Does the robot perform a on spot turn for this rotation
	 * @param angleInDeg Angle to turn in degree
	 * @param firstPoint Is this the first point on a path 
	 * @return True iff the robot would perform a on spot rotation for this path point */
	bool performsOnSpotTurn(double angleInDeg, bool firstPoint) const;

private:
	/** Calculate the on spot turning time for the robot. This includes acceleration and deceleration time 
	 * @param angle of the rotation
	 * @return The calculated on spot turning time */
	double getOnSpotTurningTime(double angle) const;
};


#endif //PROTOTYPE_ROBOTHARDWAREPROFILE_HPP
