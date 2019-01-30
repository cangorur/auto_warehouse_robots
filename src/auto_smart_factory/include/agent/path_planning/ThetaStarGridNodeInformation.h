#ifndef PROTOTYPE_THETASTARGRIDINFORMATION_HPP
#define PROTOTYPE_THETASTARGRIDINFORMATION_HPP

class GridNode;

class ThetaStarGridNodeInformation {
public:
	const GridNode* node;
	double time;
	ThetaStarGridNodeInformation* prev;

	// This is ONLY for reversal path construction
	double waitTimeAtPrev;
	
	ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, double time);
};


#endif //PROTOTYPE_THETASTARGRIDINFORMATION_HPP
