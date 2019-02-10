#ifndef PROJECT_TIMINGCALCULATOR_H
#define PROJECT_TIMINGCALCULATOR_H

#include "RobotHardwareProfile.h"
#include "ThetaStarGridNodeInformation.h"
#include "OrientedPoint.h"
#include "Point.h"

class TimingCalculator {
public:
	explicit TimingCalculator() = default;
	TimingCalculator(double startingTime, OrientedPoint startPoint, RobotHardwareProfile* hardwareProfile);
	
	double getUncertainty(double time) const;
	double getDrivingAndTurningTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const;
	double getTurningTime(Point prev, Point curr, Point next) const;
	double getTurningTime(double startRotationInDeg, Point curr, Point next) const;
	bool performsOnSpotTurn(Point prev, Point curr, Point next) const;
	bool performsOnSpotTurn(double startRotationInDeg, Point curr, Point next) const;

private:
	double startingTime;
	OrientedPoint startPoint;
	RobotHardwareProfile* hardwareProfile;
};

#endif //PROJECT_TIMINGCALCULATOR_H
