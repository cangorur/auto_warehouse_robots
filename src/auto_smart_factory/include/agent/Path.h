#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include "agent/Point.h"

class Path {

public:
    Path();
    virtual ~Path();

    std::vector<Point> points;

    void append(Point p);
    void append(std::vector<Point> points);
};

#endif /* AGENT_PATH_H_ */