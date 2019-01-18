#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include <visualization_msgs/Marker.h>
#include "agent/path_planning/Point.h"

class Path {
private:
	std::vector<Point> points;
	float length;

public:
	explicit Path(std::vector<Point> points);
	virtual ~Path() = default;

	visualization_msgs::Marker getVisualizationMsgPoints();
	visualization_msgs::Marker getVisualizationMsgLines();
	
	const std::vector<Point>& getPoints() const;
	float getLength() const;
	float getEstimatedBatteryConsumption() const; 
};

#endif /* AGENT_PATH_H_ */