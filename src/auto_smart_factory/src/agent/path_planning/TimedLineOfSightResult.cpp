#include "agent/path_planning/TimedLineOfSightResult.h"

TimedLineOfSightResult::TimedLineOfSightResult() {
	blockedByStatic = false;
	blockedByTimed = false;
	freeAfter = 0;

	hasUpcomingObstacle = false;
	lastValidEntryTime = 999999;
	freeAfterUpcomingObstacle = 0;
}
