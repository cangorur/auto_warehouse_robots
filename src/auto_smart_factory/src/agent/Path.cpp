#include <agent/Path.h>

Path::Path(std::vector<Point> points) : points(points) {

}

void Path::update(std::vector<Point> points) {
    points = points;
}

const std::vector<Point>& Path::getPoints() {
    return points;
}

Path::~Path() {

}