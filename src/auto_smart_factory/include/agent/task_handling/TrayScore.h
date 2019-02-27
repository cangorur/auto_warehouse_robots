#ifndef AGENT_TRAYSCORE_H_
#define AGENT_TRAYSCORE_H_

#include "ros/ros.h"

/**
 * Class for holding data to compare between different source and target tray pairs
 */
class TrayScore {
public:
	TrayScore(uint32_t sourceTray, uint32_t targetTray, double score, double estimatedDuration);
	virtual ~TrayScore() = default;

	// the tray id of the source tray
	uint32_t sourceTray;
	// the tray id of the target tray
	uint32_t targetTray;
	// the calculated score between robot position and source_tray as well as source_tray and target_tray
	double score;
	// the estimated duration between robot position and source_tray as well as source_tray and target_tray
	double estimatedDuration;
};

// this operator checks if all members of the TrayScores are equal
bool operator ==(const TrayScore& left, const TrayScore& right);

#endif