#ifndef AGENT_ORIENTEDPOINT_H_
#define AGENT_ORIENTEDPOINT_H_

class OrientedPoint {
public:
	explicit OrientedPoint();
	explicit OrientedPoint(float x, float y, float o);
    virtual ~OrientedPoint();

    float x;
    float y;
    float o;
};

#endif /* AGENT_ORIENTEDPOINT_H_ */