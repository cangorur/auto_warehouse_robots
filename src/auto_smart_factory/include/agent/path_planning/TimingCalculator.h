#ifndef PROJECT_TIMINGCALCULATOR_H
#define PROJECT_TIMINGCALCULATOR_H

#include "RobotHardwareProfile.h"
#include "ThetaStarGridNodeInformation.h"
#include "OrientedPoint.h"

class TimingCalculator {
public:
	explicit TimingCalculator() = default;
	TimingCalculator(double startingTime, RobotHardwareProfile* hardwareProfile);
	TimingCalculator(double startingTime, OrientedPoint startPoint, RobotHardwareProfile* hardwareProfile);
	
	double getUncertainty(double time) const;
	double getDrivingTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const;
	//double getDrivingTime()

private:
	double startingTime;
	OrientedPoint startPoint;
	RobotHardwareProfile* hardwareProfile;
};

#endif //PROJECT_TIMINGCALCULATOR_H
