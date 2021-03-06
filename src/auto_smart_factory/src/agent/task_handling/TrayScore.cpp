#include "agent/task_handling/TrayScore.h"

TrayScore::TrayScore(uint32_t sourceTray, uint32_t targetTray, double score, double estimatedDuration) :
	sourceTray(sourceTray), 
	targetTray(targetTray),
	score(score),
	estimatedDuration(estimatedDuration)
{
}

bool operator ==(const TrayScore& left, const TrayScore& right){
	return (left.sourceTray == right.sourceTray && left.targetTray == right.targetTray && left.score == right.score);
}