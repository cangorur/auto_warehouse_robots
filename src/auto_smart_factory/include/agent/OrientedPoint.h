#ifndef AGENT_ORIENTEDPOINT_H_
#define AGENT_ORIENTEDPOINT_H_

class Point
{

  public:
    Point(float x = 0.0f, float y = 0.0f, float o = 0.0f);
    virtual ~Point();

    float x;
    float y;
    float o;
};

#endif /* AGENT_ORIENTEDPOINT_H_ */