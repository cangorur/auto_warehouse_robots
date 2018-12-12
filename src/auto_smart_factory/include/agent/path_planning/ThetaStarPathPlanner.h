#ifndef PROTOTYPE_THETASTARPATHPLANNER_HPP
#define PROTOTYPE_THETASTARPATHPLANNER_HPP

#include <queue>

#include "Math.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/ThetaStarGridNodeInformation.h"

#define USE_THETA_STAR true

class ThetaStarPathPlanner {
private:
	typedef std::pair<float, ThetaStarGridNodeInformation*> GridInformationPair;
	struct GridInformationPairComparator {
		bool operator()(GridInformationPair const& lhs, GridInformationPair const& rhs){
			return lhs.first > rhs.first;
		}
	};
	typedef std::priority_queue<GridInformationPair, std::vector<GridInformationPair>, GridInformationPairComparator> GridInformationPairQueue;
	
	typedef std::map<Point, ThetaStarGridNodeInformation, Math::PointComparator> GridInformationMap;

	ThetaStarMap* map;

public:
	explicit ThetaStarPathPlanner(ThetaStarMap* thetaStarMap);

	Path findPath(Point start, Point target);

private:
	float getHeuristic(Point point, Point target) const;
};


#endif //PROTOTYPE_THETASTARPATHPLANNER_HPP
