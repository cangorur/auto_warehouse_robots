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
	explicit ThetaStarPathPlanner(ThetaStarMap* thetaStarMap, RobotHardwareProfile* hardwareProfile, OrientedPoint start, OrientedPoint target, double startingTime, double targetReservationTime);
	
	Path findPath();

private:
	double initialTime = std::numeric_limits<double>::max() - 100000;
	double desiredDistanceForCurveEdge = 0.85f;

	typedef std::pair<double, ThetaStarGridNodeInformation*> GridInformationPair;
	struct GridInformationPairComparator {
		bool operator()(GridInformationPair const& lhs, GridInformationPair const& rhs){
			return lhs.first > rhs.first;
		}
	};
	typedef std::priority_queue<GridInformationPair, std::vector<GridInformationPair>, GridInformationPairComparator> GridInformationPairQueue;
	typedef std::map<Point, ThetaStarGridNodeInformation, Math::PointComparator> GridInformationMap;

	// Functions	
	double getHeuristic(ThetaStarGridNodeInformation* current, Point targetPos) const;
	Path constructPath(double startingTime, ThetaStarGridNodeInformation* targetInformation, double targetReservationTime) const;	

	// Smoothing
	Path smoothPath(Path source) const;
	bool shouldSmoothCorner(Point prev, Point curr, Point next, double waitTimeAtCenter) const;

	// Return curve which replaces the center input point
	std::vector<Point> createCurveFromCorner(Point prev, Point curr, Point next, Point lastPointInOutput) const;
	std::vector<Point> addPointsToCurve(Point curveStart, Point center, Point curveEnd, int pointsToAdd) const;
	std::vector<Point> smoothCurve(const std::vector<Point>& input) const;

	Point getCurveEdge(Point neighbour, Point center) const;
	Point getCurveEdge(Point neighbour, Point center, Point lastPointInOutput) const;
	double getAngle(const Point& prev, const Point& curr, const Point& next) const;

	// Query information
	ThetaStarMap* map;
	RobotHardwareProfile* hardwareProfile;
	TimingCalculator timing;
	
	OrientedPoint start;
	OrientedPoint target;
	const GridNode* startNode;
	const GridNode* targetNode;	
	double startingTime;
	double targetReservationTime;
	bool isValidPathQuery;
	
	std::vector<Rectangle> reservationsToIgnore;
};


#endif //PROTOTYPE_THETASTARPATHPLANNER_HPP
