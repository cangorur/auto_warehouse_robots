#include "agent/path_planning/ThetaStarGridNodeInformation.h"

ThetaStarGridNodeInformation::ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, double time) :
		node(node),
		time(time),
		prev(prev),
		waitTimeAtPrev(0) 
{}
