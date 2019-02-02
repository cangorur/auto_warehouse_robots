#ifndef AGENT_TRAYSCORE_H_
#define AGENT_TRAYSCORE_H_

#include "ros/ros.h"

class TrayScore {
public:
	TrayScore(uint32_t sourceTray, uint32_t targetTray, double score, double estimatedDuration);
	virtual ~TrayScore() = default;

	// the tray id of the source tray
	uint32_t sourceTray;
	// the tray id of the target tray
	uint32_t targetTray;
	// the calculated score between source_tray and target_tray
	double score;		
	double estimatedDuration;
};

// this operator checks if all members of the TrayScores are equal
bool operator ==(const TrayScore& left, const TrayScore& right);

#endif