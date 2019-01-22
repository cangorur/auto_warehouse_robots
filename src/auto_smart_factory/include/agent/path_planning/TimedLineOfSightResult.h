#ifndef PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP
#define PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP


class TimedLineOfSightResult {
public:
	bool blockedByStatic;
	bool blockedByTimed;
	float freeAfter;
	
	bool hasUpcomingObstacle;
	float lastValidEntryTime;
	float freeAfterUpcomingObstacle;
	
	TimedLineOfSightResult();
};


#endif //PROTOTYPE_TIMEDLINEOFSIGHTRESULT_HPP
