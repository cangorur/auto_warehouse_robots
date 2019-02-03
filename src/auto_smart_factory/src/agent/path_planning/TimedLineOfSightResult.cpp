#include "agent/path_planning/TimedLineOfSightResult.h"

TimedLineOfSightResult::TimedLineOfSightResult() {
	blockedByStatic = false;
	blockedByTimed = false;
	freeAfter = 0;

	hasUpcomingObstacle = false;
	lastValidEntryTime = 1000000000000;
	freeAfterUpcomingObstacle = 0;
}
