#include "agent/path_planning/TimingCalculator.h"
#include "agent/path_planning/GridNode.h"
#include <cmath>
#include <include/agent/path_planning/TimingCalculator.h>

#include "Math.h"

using namespace UncertaintyDirection;

TimingCalculator::TimingCalculator(double startingTime, OrientedPoint startPoint, RobotHardwareProfile* hardwareProfile) :
	startingTime(startingTime),
	startPoint(startPoint),
	hardwareProfile(hardwareProfile)
{
}

double TimingCalculator::getPlanningUncertainty(double time, Direction direction) const {
	return 0;
	double uncertainty = getUncertainty(time);

	if(direction == Direction::BEHIND) {
		return uncertainty * 0.1f;
	} else {
		return uncertainty * 1.0f;
	}
}

double TimingCalculator::getReservationUncertainty(double time, Direction direction) const {
	double uncertainty = getUncertainty(time);
	
	if(direction == Direction::BEHIND) {
		return uncertainty * 0.3f;
	} else {
		return uncertainty * 0.6f;
	}
}

double TimingCalculator::getDrivingAndTurningTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const {
	double turningTime = 0;

	// Include turningTime to current line segment if prev is available
	if(current->prev != nullptr) {
		turningTime = getTurningTime(current->prev->node->pos, current->node->pos, target->node->pos);
	} else {
		turningTime = getTurningTime(startPoint.o, current->node->pos, target->node->pos);
	}

	return turningTime + hardwareProfile->getDrivingDuration(Math::getDistance(current->node->pos, target->node->pos));
}

double TimingCalculator::getTurningTime(Point prev, Point curr, Point next) const {
	double prevLineSegmentRotation = Math::getRotationInDeg(curr - prev);
	double currLineSegmentRotation = Math::getRotationInDeg(next - curr);
	
	return hardwareProfile->getTurningDuration(std::abs(Math::getAngleDifferenceInDegree(prevLineSegmentRotation, currLineSegmentRotation)));
}

double TimingCalculator::getTurningTime(double startRotationInDeg, Point curr, Point next) const {
	double currLineSegmentRotation = Math::getRotationInDeg(next - curr);

	return hardwareProfile->getTurningDuration(std::abs(Math::getAngleDifferenceInDegree(startRotationInDeg, currLineSegmentRotation)));
}

bool TimingCalculator::performsOnSpotTurn(Point prev, Point curr, Point next, bool firstPoint) const {
	double prevLineSegmentRotation = Math::getRotationInDeg(curr - prev);
	double currLineSegmentRotation = Math::getRotationInDeg(next - curr);

	return hardwareProfile->performsOnSpotTurn(std::abs(Math::getAngleDifferenceInDegree(prevLineSegmentRotation, currLineSegmentRotation)), firstPoint);
}

bool TimingCalculator::performsOnSpotTurn(double startRotationInDeg, Point curr, Point next,bool firstPoint) const {
	double currLineSegmentRotation = Math::getRotationInDeg(next - curr);

	return hardwareProfile->performsOnSpotTurn(std::abs(Math::getAngleDifferenceInDegree(startRotationInDeg, currLineSegmentRotation)), firstPoint);
}

double TimingCalculator::getRelativeUncertainty(double time) const {
	double timeSinceStart = time - startingTime;
	if(timeSinceStart <= 0) {
		return 0;
	}
	double uncertainty = timeSinceStart * hardwareProfile->getTimeUncertaintyPercentage();
	return std::abs(uncertainty);
}

double TimingCalculator::getUncertainty(double time) const {
	double timeSinceStart = time - startingTime;
	if(timeSinceStart <= 0) {
		return 0;
	}
	double uncertainty = timeSinceStart * hardwareProfile->getTimeUncertaintyPercentage() + hardwareProfile->getTimeUncertaintyAbsolute();
	return std::abs(uncertainty);
}

