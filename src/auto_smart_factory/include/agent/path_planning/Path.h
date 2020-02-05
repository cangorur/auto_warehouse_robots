#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include "visualization_msgs/Marker.h"
#include "agent/path_planning/Point.h"
#include "agent/path_planning/Rectangle.h"
#include "RobotHardwareProfile.h"
#include "TimingCalculator.h"

/* Class which represents a calculated path object. This path might be invalid because no path was found for a specific path query.
 * ALWAYS check path.isValid() before using it! */
class Path {
public:
	// Max duration a reservation (which is not a waiting reservation) may last
	double maxDrivingReservationDuration = 2.0f;
	
	// Duration Margin a reservations lasts longer than necessary to add safety leeway
	double reservationTimeMarginAhead = 0.3f;
	double reservationTimeMarginBehind = 0.2f;
	
	// Additional time included to approach the final path point due to motion planner precision mode
	double finalPointAdditionalTime = 0.5f;
	
private:
	// The starting time of this path
	double startTimeOffset = 0;
	
	// Path nodes
	std::vector<Point> nodes;
	
	// Wait times of the path nodes
	std::vector<double> waitTimes;
	
	// Hardware profile of the robots this path is for
	RobotHardwareProfile* hardwareProfile;
	
	// The first path point inclusive orientation
	OrientedPoint start;

	// The last path point inclusive orientation
	OrientedPoint end;
	
	// Is this path a valid path?
	bool isValidPath;

	// ==== Computed internally for motion planner ====
	// Time stamp when the Motion Planner is allowed to leave a path node
	std::vector<double> departureTimes;
	
	// Path distance
	double distance;
	
	// Path duration
	double duration;
	
	// Estimated battery consumption
	double batteryConsumption;
	
	// Timing Calculator used for this path
	TimingCalculator timing;
	
	// Reservation duration at the final path point
	double targetReservationTime;
	
	// The estimated driving time for each path segment
	std::vector<double> drivingTimes;
	
	// Estimated onSpot times for each path point. OnSpot includes waiting times and turning times if a onSpot rotation is performed
	std::vector<double> onSpotTimes;

public:
	explicit Path();
	/** Constructor
	 * @param startTimeOffset the starting time of this path
	 * @param nodes list of path points
	 * @param waitTimes the waiting times at each path point
	 * @param hardwareProfile The hardware profile the robot this path is for uses
	 * @param targetReservationTime The reservation duration at the final path point
	 * @param start Path start Point
	 * @param end Path end point
	 * @param agentId The agent id this path belongs to
	 * */
	explicit Path(double startTimeOffset, std::vector<Point> nodes, std::vector<double> waitTimes, RobotHardwareProfile* hardwareProfile, double targetReservationTime, OrientedPoint start, OrientedPoint end, int agentId);
	
	virtual ~Path() = default;

	// Getters
	const std::vector<Point>& getNodes() const;
	const std::vector<double>& getWaitTimes() const;
	const std::vector<double>& getDepartureTimes() const;

	double getDistance() const;
	double getDuration() const;
	double getBatteryConsumption() const;
	double getStartTimeOffset() const;
	RobotHardwareProfile* getRobotHardwareProfile() const;

	OrientedPoint getStart() const;
	OrientedPoint getEnd() const;
	bool isValid() const;

	/** Compute the reservations size based on the robot size 
	 * @return the reservations size */
	static double getReservationSize();

	/** Generate the reservations for this path
	 * @param ownerId The owner id of the agent the reservations shall belong to
	 * @param startsAtTray True iff the Path starts at a tray (as opposed to a point somewhere else on the map). If true, larger reservations in front of this tray are generated too
	 * @return the generated reservations
	 * */
	const std::vector<Rectangle> generateReservations(int ownerId, bool startsAtTray) const;
	
	/** Generate RVIZ visualisation marker message
	 * @param color the color the markers should have
	 * @return The visualisation marker message */
	visualization_msgs::Marker getVisualizationMsgLines(std_msgs::ColorRGBA color);

private:
	/** Generate reservations for a straight line segment
	 * @param reservations The reservations vector to add these reservations to
	 * @param startPoint Line segment start point
	 * @param endPoint Line segment end point
	 * @param timeAtStartPoint TimeStamp for the start point
	 * @param deltaDuration Duration for this line segment
	 * @param ownerId Owner id for the generated reservations
	 * */
	void generateReservationsForSegment(std::vector<Rectangle>& reservations, Point startPoint, Point endPoint, double timeAtStartPoint, double deltaDuration, int ownerId) const;

	/** Generate reservations for a curved line segment
	 * @param reservations The reservations vector to add these reservations to
	 * @param points List of Points describing the curved line segment
	 * @param timeAtStartPoint TimeStamp for the start point
	 * @param deltaDuration Duration for this line segment
	 * @param ownerId Owner id for the generated reservations
	 * */
	void generateReservationsForCurvePoints(std::vector<Rectangle>& reservations, std::vector<Point> points, double timeAtStartPoint, double deltaTime, int ownerId) const;
	
	/** Generate reservations in front of a tray
	 * @param reservations The reservations vector to add these reservations to
	 * @param pos the Position in front of the tray
	 * @param reservationStartTime Start time of the reservations
	 * @param duration Duration of the reservations
	 * @param ownerId Owner id for the generated reservations
	 * */
	void generateReservationForTray(std::vector<Rectangle>& reservations, OrientedPoint pos, double reservationStartTime, double duration, int ownerId) const;
};

#endif /* AGENT_PATH_H_ */