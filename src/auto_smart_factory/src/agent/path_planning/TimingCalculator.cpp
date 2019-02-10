#include "agent/path_planning/TimingCalculator.h"
#include <cmath>
#include <include/agent/path_planning/TimingCalculator.h>

#include "Math.h"
#include "agent/path_planning/GridNode.h"


TimingCalculator::TimingCalculator(double startingTime, OrientedPoint startPoint, RobotHardwareProfile* hardwareProfile) :
	startingTime(startingTime),
	startPoint(startPoint),
	hardwareProfile(hardwareProfile)
{
}

TimingCalculator::TimingCalculator(double startingTime, RobotHardwareProfile* hardwareProfile) :
	TimingCalculator(startingTime, OrientedPoint(0, 0, 0), hardwareProfile)
{
}

double TimingCalculator::getUncertainty(double time) const {
	double timeSinceStart = time - startingTime;
	if(timeSinceStart <= 0) {
		return 0;
	}
	double uncertainty = timeSinceStart * hardwareProfile->getTimeUncertaintyPercentage() + hardwareProfile->getTimeUncertaintyAbsolute();
	return std::abs(uncertainty);
}

double TimingCalculator::getDrivingTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const {
	// Include turningTime to current line segment if prev is available
	double turningTime = 0;

	double prevLineSegmentRotation = 0;
	if(current->prev != nullptr) {
		prevLineSegmentRotation = Math::getRotationInDeg(current->node->pos - current->prev->node->pos);
	} else {
		prevLineSegmentRotation = startPoint.o;
	}

	double currLineSegmentRotation = Math::getRotationInDeg(target->node->pos - current->node->pos);
	turningTime = hardwareProfile->getTurningDuration(std::abs(Math::getAngleDifferenceInDegree(prevLineSegmentRotation, currLineSegmentRotation)));

	return hardwareProfile->getDrivingDuration(Math::getDistance(current->node->pos, target->node->pos)) + turningTime;
}
