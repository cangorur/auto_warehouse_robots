#include "agent/path_planning/ThetaStarGridNodeInformation.h"

ThetaStarGridNodeInformation::ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, float time) :
		node(node),
		time(time),
		prev(prev),
		waitTimeAtPrev(0) 
{}
