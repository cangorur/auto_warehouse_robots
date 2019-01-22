#ifndef PROTOTYPE_THETASTARGRIDINFORMATION_HPP
#define PROTOTYPE_THETASTARGRIDINFORMATION_HPP

class GridNode;

class ThetaStarGridNodeInformation {
public:
	const GridNode* node;
	float time;
	ThetaStarGridNodeInformation* prev;

	// This is ONLY for reversal path construction
	float waitTimeAtPrev;
	
	ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, float time);
};


#endif //PROTOTYPE_THETASTARGRIDINFORMATION_HPP
