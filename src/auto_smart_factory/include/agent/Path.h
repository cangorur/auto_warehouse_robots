#ifndef AGENT_PATH_H_
#define AGENT_PATH_H_

#include <tuple>
#include <list>

class Path {

public:
    Path();
    virtual ~Path();

private:
    std::list<std::tuple<float, float>> points;

};

#endif /* AGENT_PATH_H_ */