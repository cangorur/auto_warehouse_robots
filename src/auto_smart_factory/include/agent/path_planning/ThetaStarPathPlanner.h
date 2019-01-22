#ifndef PROTOTYPE_THETASTARPATHPLANNER_HPP
#define PROTOTYPE_THETASTARPATHPLANNER_HPP

#include <queue>

#include "Math.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/ThetaStarGridNodeInformation.h"
#include "RobotHardwareProfile.h"

#define USE_THETA_STAR true

class ThetaStarPathPlanner {
private:
	float initialTime = std::numeric_limits<float>::max() - 10000;
	
	typedef std::pair<float, ThetaStarGridNodeInformation*> GridInformationPair;
	struct GridInformationPairComparator {
		bool operator()(GridInformationPair const& lhs, GridInformationPair const& rhs){
			return lhs.first > rhs.first;
		}
	};
	typedef std::priority_queue<GridInformationPair, std::vector<GridInformationPair>, GridInformationPairComparator> GridInformationPairQueue;
	typedef std::map<Point, ThetaStarGridNodeInformation, Math::PointComparator> GridInformationMap;

	ThetaStarMap* map;
	RobotHardwareProfile* hardwareProfile;

public:
	explicit ThetaStarPathPlanner(ThetaStarMap* thetaStarMap, RobotHardwareProfile* hardwareProfile);

	Path findPath(Point start, Point target, float startingTime);

private:
	float getHeuristic(ThetaStarGridNodeInformation* current, Point targetPos) const;
	float getDrivingTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const;

	Path constructPath(float startingTime, ThetaStarGridNodeInformation* targetInformation) const;
};


#endif //PROTOTYPE_THETASTARPATHPLANNER_HPP
