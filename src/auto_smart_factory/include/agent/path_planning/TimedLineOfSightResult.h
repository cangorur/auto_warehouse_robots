#ifndef PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP
#define PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP


class TimedLineOfSightResult {
public:
	bool blockedByStatic;
	bool blockedByTimed;
	double freeAfter;
	
	bool hasUpcomingObstacle;
	double lastValidEntryTime;
	double freeAfterUpcomingObstacle;
	
	TimedLineOfSightResult();
};


#endif //PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP
