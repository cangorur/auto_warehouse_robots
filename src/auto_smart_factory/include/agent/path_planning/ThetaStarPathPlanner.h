#ifndef PROTOTYPE_THETASTARPATHPLANNER_HPP
#define PROTOTYPE_THETASTARPATHPLANNER_HPP

#include <queue>

#include "Math.h"
#include "agent/path_planning/ThetaStarMap.h"
#include "agent/path_planning/Path.h"
#include "agent/path_planning/ThetaStarGridNodeInformation.h"
#include "RobotHardwareProfile.h"

// This class represents a Theta* Path Planner. The search query parameters are specified with the constructor, the path planner is intended to be used only once. A new one needs to be constructed for every path query
class ThetaStarPathPlanner {
public:
	/** Constructor for a new path query
	 * @param thetaStarMap ThetaStar Mao to search on
	 * @param hardwareProfile HardwareProfile for the robot the path if for
	 * @param start Start point with orientation
	 * @param target Target point with orientation
	 * @param startingTime Start Time of the path
	 * @param targetReservationTime Duration of the reservations at the path target
	 * @param ignoreStartingReservations Should reservations at the starting position be ignored? */
	explicit ThetaStarPathPlanner(ThetaStarMap* thetaStarMap, RobotHardwareProfile* hardwareProfile, OrientedPoint start, OrientedPoint target, double startingTime, double targetReservationTime, bool ignoreStartingReservations);
	
	Path findPath();

private:
	// Time value for unexplored Theta* Grid Nodes
	double initialTime = std::numeric_limits<double>::max() - 100000;
	
	// Distance from the first curved point fro the curve corner for path smoothing 
	double desiredDistanceForCurveEdge = 0.85f;

	// GridNode Comparator
	typedef std::pair<double, ThetaStarGridNodeInformation*> GridInformationPair;
	struct GridInformationPairComparator {
		bool operator()(GridInformationPair const& lhs, GridInformationPair const& rhs){
			return lhs.first > rhs.first;
		}
	};
	
	// Typedef for the Theta* Queue
	typedef std::priority_queue<GridInformationPair, std::vector<GridInformationPair>, GridInformationPairComparator> GridInformationPairQueue;
	
	// Typedef for the Theta* Collection of already explored nodes
	typedef std::map<Point, ThetaStarGridNodeInformation, Math::PointComparator> GridInformationMap;

	/** Computes the heuristic for a specific position
	 * @param current The current position
	 * @param targetPos Target position of the current search query
	 * @return Returns the computed heuristic */
	double getHeuristic(ThetaStarGridNodeInformation* current, Point targetPos) const;
	
	/** Construct the path from the chain of previous pointers after theta* completion
	 * @param startingTime Starting Time for the path 
	 * @param targetInformation GridNode Information of the found path planning target
	 * @param targetReservationTime Duration of the reservation at the path target 
	 * @return The constructed Path */
	Path constructPath(double startingTime, ThetaStarGridNodeInformation* targetInformation, double targetReservationTime) const;	

	/** Smooth a given path
	 * @param source Path to smooth 
	 * @return The smoothed path */
	Path smoothPath(Path source) const;
	
	/**
	 * Determines if a path corner should be smoothed
	 * @param prev Previous path point
	 * @param curr Current path point
	 * @param next Next path point
	 * @param waitTimeAtCenter waiting time at the current path position 
	 * @return True iff the corner should be smoothed */
	bool shouldSmoothCorner(Point prev, Point curr, Point next, double waitTimeAtCenter) const;

	// Return curve which replaces the center input point
	/** 
	 * @param prev 
	 * @param curr 
	 * @param next 
	 * @param lastPointInOutput 
	 * @return */
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
	
	std::vector<Rectangle> smallerReservations;
};


#endif //PROTOTYPE_THETASTARPATHPLANNER_HPP

