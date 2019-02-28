#ifndef PROTOTYPE_THETASTARGRIDINFORMATION_HPP
#define PROTOTYPE_THETASTARGRIDINFORMATION_HPP

class GridNode;

/* Class which represents a Theta* grid node during a theta* path computation. Contains the GridNode, exploration time and previous node (for path construction after the target was found) */
class ThetaStarGridNodeInformation {
public:
	const GridNode* node;
	double time;
	ThetaStarGridNodeInformation* prev;

	// Waiting time at the previous path node. This is ONLY for reversal path construction
	double waitTimeAtPrev;
	
	ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, double time);
};


#endif //PROTOTYPE_THETASTARGRIDINFORMATION_HPP
