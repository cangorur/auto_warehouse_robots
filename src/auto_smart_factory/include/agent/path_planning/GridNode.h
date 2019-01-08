#ifndef PROTOTYPE_GRIDNODE_HPP
#define PROTOTYPE_GRIDNODE_HPP

#include <vector>

#include "agent/path_planning/Point.h"

class GridNode {
public:
	Point pos;
	std::vector<GridNode*> neighbours;

	explicit GridNode(Point pos);
};

#endif //PROTOTYPE_GRIDNODE_HPP
