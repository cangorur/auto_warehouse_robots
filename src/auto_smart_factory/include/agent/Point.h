#ifndef AGENT_POINT_H_
#define AGENT_POINT_H_

class Point
{

  public:
    Point(float x = 0.0f, float y = 0.0f);
    virtual ~Point();

    float x;
    float y;
};

#endif /* AGENT_POINT_H_ */