#ifndef PROJECT_TIMINGCALCULATOR_H
#define PROJECT_TIMINGCALCULATOR_H

#include "RobotHardwareProfile.h"
#include "ThetaStarGridNodeInformation.h"
#include "OrientedPoint.h"
#include "Point.h"

// Define special namespace with enum class for more sophisticated parameters. Also don't clutter public namespace
namespace UncertaintyDirection {
	enum class Direction {BEHIND, AHEAD};	
}
using namespace UncertaintyDirection;

// Utility class to perform timing calculations
class TimingCalculator {
public:	
	explicit TimingCalculator() = default;
	
	/** Constructor
	 * @param startingTime The starting time of the path query 
	 * @param startPoint The starting point of the path query
	 * @param hardwareProfile The used hardware profile of the robot */
	TimingCalculator(double startingTime, OrientedPoint startPoint, RobotHardwareProfile* hardwareProfile);
	
	/** Calculate time uncertainty for planning
	 * @param time start time
	 * @param direction Uncertainty behind or in front of the current time?
	 * @return calculated timing uncertainty for planning */
	double getPlanningUncertainty(double time, Direction direction) const;

	/** Calculate time uncertainty for reserving
	 * @param time start time
	 * @param direction Uncertainty behind or in front of the current time?
	 * @return calculated timing uncertainty for reserving */
	double getReservationUncertainty(double time, Direction direction) const;

	/** Calculate relative time uncertainty
	 * @param time start time
	 * @return calculated relative timing uncertainty */
	double getRelativeUncertainty(double time) const;
	
	/** Calculate estimated time for driving and turning to the next path node
	 * @param current Current path node
	 * @param target Target path node
	 * @return estimated driving + turning time */
	double getDrivingAndTurningTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const;
	
	/** Estimate turning time until next line segment
	 * @param prev Previous line segment start
	 * @param curr Current position
	 * @param next Next line segment to rotate towards
	 * @return Estimated turning time */
	double getTurningTime(Point prev, Point curr, Point next) const;

	/** Estimate turning time until next line segment
	 * @param startRotationInDeg Current rotation
	 * @param curr Current position
	 * @param next Next line segment to rotate towards
	 * @return Estimated turning time */
	double getTurningTime(double startRotationInDeg, Point curr, Point next) const;
	
	/** Does the robot plan to perform a on spot turn for this corner?
	 * @param prev Previous line segment start
	 * @param curr Current position
	 * @param next Next line segment to rotate towards  
	 * @param firstPoint Is this the first path point?
	 * @return True iff a on spot turn will be performed for this corner */
	bool performsOnSpotTurn(Point prev, Point curr, Point next, bool firstPoint) const;

	/** Does the robot plan to perform a on spot turn for this corner?
	 * @param startRotationInDeg Current rotation
	 * @param curr Current position
	 * @param next Next line segment to rotate towards  
	 * @param firstPoint Is this the first path point?
	 * @return True iff a on spot turn will be performed for this corner */
	bool performsOnSpotTurn(double startRotationInDeg, Point curr, Point next, bool firstPoint) const;

private:
	// Path starting time
	double startingTime;
	
	// Path starting point with orientation
	OrientedPoint startPoint;
	
	// Robots hardware profile used for estimating timings and battery consumption
	RobotHardwareProfile* hardwareProfile;
	
	// Internal uncertainty computation used by the public ones
	double getUncertainty(double time) const;
};

#endif //PROJECT_TIMINGCALCULATOR_H
