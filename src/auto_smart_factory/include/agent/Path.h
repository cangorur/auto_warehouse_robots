#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <vector>
#include "agent/Point.h"

class Path {

public:
  Path(std::vector<Point> points = {});
  virtual ~Path();

  void update(std::vector<Point> points);
  const std::vector<Point> getPoints(void);

private:
    std::vector<Point> points;
};

#endif /* AGENT_PATH_H_ */