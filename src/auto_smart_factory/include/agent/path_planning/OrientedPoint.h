#ifndef AGENT_ORIENTEDPOINT_H_
#define AGENT_ORIENTEDPOINT_H_

class OrientedPoint {
public:
	explicit OrientedPoint();
	explicit OrientedPoint(double x, double y, double o);

	virtual ~OrientedPoint() = default;

	double x;
	double y;
	double o;
};

#endif /* AGENT_ORIENTEDPOINT_H_ */