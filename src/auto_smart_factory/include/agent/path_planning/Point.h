#ifndef AGENT_POINT_H_
#define AGENT_POINT_H_

class Point {
public:
	explicit Point();

	explicit Point(float x, float y);

	virtual ~Point();

	float x;
	float y;
};

#endif /* AGENT_POINT_H_ */