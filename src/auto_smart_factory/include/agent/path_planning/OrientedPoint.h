#ifndef AGENT_ORIENTEDPOINT_H_
#define AGENT_ORIENTEDPOINT_H_

/* Class which represents a 2D point with an orientation in Rad */
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