#ifndef PROTOTYPE_THETASTARGRIDINFORMATION_HPP
#define PROTOTYPE_THETASTARGRIDINFORMATION_HPP

class GridNode;

class ThetaStarGridNodeInformation {
public:
	const GridNode* node;
	float g;
	ThetaStarGridNodeInformation* prev;
	ThetaStarGridNodeInformation* expandedBy;
	
	ThetaStarGridNodeInformation(const GridNode* node, ThetaStarGridNodeInformation* prev, float g);
};


#endif //PROTOTYPE_THETASTARGRIDINFORMATION_HPP
