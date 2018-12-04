#include <utility>

#include <agent/path_planning/Path.h>
#include <include/agent/path_planning/Path.h>

Path::Path(std::vector<Point> points_) :
	points(std::move(points_)) {

	for(Point& point : points) {
		// TODO getDistance...
	}
}

const std::vector<Point>& Path::getPoints() const {
	return points;
}

float Path::getLength() const {
	return length;
}
