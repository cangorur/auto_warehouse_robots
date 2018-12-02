#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include "geometry_msgs/Point.h"

class Path {

public:
    Path();
    virtual ~Path();

    std::vector<geometry_msgs::Point> points;

    void addPoint(geometry_msgs::Point p);

};

#endif /* AGENT_PATH_H_ */