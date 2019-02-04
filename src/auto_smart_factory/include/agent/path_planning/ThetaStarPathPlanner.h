#ifndef PROTOTYPE_THETASTARPATHPLANNER_HPP
#define PROTOTYPE_THETASTARPATHPLANNER_HPP

#include <queue>

#include "Math.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/ThetaStarGridNodeInformation.h"
#include "RobotHardwareProfile.h"

class ThetaStarPathPlanner {
public:
	explicit ThetaStarPathPlanner(ThetaStarMap* thetaStarMap, RobotHardwareProfile* hardwareProfile);
	Path findPath(OrientedPoint start, OrientedPoint target, double startingTime, double targetReservationTime);

private:
	double initialTime = std::numeric_limits<double>::max() - 100000;

	typedef std::pair<double, ThetaStarGridNodeInformation*> GridInformationPair;
	struct GridInformationPairComparator {
		bool operator()(GridInformationPair const& lhs, GridInformationPair const& rhs){
			return lhs.first > rhs.first;
		}
	};
	typedef std::priority_queue<GridInformationPair, std::vector<GridInformationPair>, GridInformationPairComparator> GridInformationPairQueue;
	typedef std::map<Point, ThetaStarGridNodeInformation, Math::PointComparator> GridInformationMap;

	ThetaStarMap* map;
	RobotHardwareProfile* hardwareProfile;
	
	double getHeuristic(ThetaStarGridNodeInformation* current, Point targetPos) const;
	double getDrivingTime(ThetaStarGridNodeInformation* current, ThetaStarGridNodeInformation* target) const;

	Path constructPath(double startingTime, ThetaStarGridNodeInformation* targetInformation, double initialWaitTime, double targetReservationTime) const;
	
	OrientedPoint startPoint;
	OrientedPoint endPoint;
};


#endif //PROTOTYPE_THETASTARPATHPLANNER_HPP
