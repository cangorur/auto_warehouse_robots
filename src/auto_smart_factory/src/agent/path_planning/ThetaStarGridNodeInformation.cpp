#include "agent/path_planning/ThetaStarGridNodeInformation.h"

ThetaStarGridNodeInformation::ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, float g) :
		node(node),
		g(g),
		prev(prev),
		expandedBy(nullptr) {
}
