#ifndef PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP
#define PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP

// This class collects the data gained from a timed line of sight check
class TimedLineOfSightResult {
public:
	// Was the check blocked by a static obstacle?
	bool blockedByStatic;

	// Was the check blocked by a timed obstacle?
	bool blockedByTimed;
	
	// Time after which this line of sight is free
	double freeAfter;
	
	// Is there any upcoming obstacle on this line of sight?
	bool hasUpcomingObstacle;
	
	// Last valid point in time to enter this line of sight check
	double lastValidEntryTime;
	
	// Time point after which this line of sight check is free after an upcoming obstacle
	double freeAfterUpcomingObstacle;
	
	TimedLineOfSightResult();
};


#endif //PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP
