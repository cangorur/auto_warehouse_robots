#include "agent/ObstacleDetection.h"

ObstacleDetection::ObstacleDetection(std::string agent_id, MotionPlanner& motion_planner,
                                     auto_smart_factory::RobotConfiguration robot_config,
                                     auto_smart_factory::WarehouseConfiguration warehouse_config) {
	agentID = agent_id;
	robotConfig = robot_config;
	motionPlanner = &motion_planner;
	warehouseConfig = warehouse_config;

	//build grid matrix
	/*int index = 0;
	for(int i = 0; i < warehouseConfig.occupancy_map.info.height; i++) {
		std::vector<float> row;
		for(int j = 0; j < warehouseConfig.occupancy_map.info.width; j++) {
			row.push_back(warehouseConfig.occupancy_map.data[index]);
			index++;
		}
		occupancyGrid.push_back(row);
	}*/

	//initialize random seed for rand()
	srand(time(NULL));
}

ObstacleDetection::~ObstacleDetection() {
}

void ObstacleDetection::update(geometry_msgs::Point& position, double orientation,
                               const sensor_msgs::LaserScan& msg) {
	this->position = position;
	this->orientation = orientation;
	if(enabled && !motionPlanner->isDone()) {
		//random laser range
		setNextAnalyzeRange();
		//dont analyse laser data when driving backwards or when obstacles are too far away
		if(!motionPlanner->isDrivingBackwards() && isRangeSmaller(analyzeRange, msg.ranges)) {
			analyzeLaserScan(position, orientation, msg);
		} else {
			motionPlanner->start();
		}
	}
}

void ObstacleDetection::enable(bool enable) {
	enabled = enable;
}

bool ObstacleDetection::isEnabled() {
	return enabled;
}

void ObstacleDetection::setNextAnalyzeRange() {
	//analyzeRange = analyzeRangeMin + (rand() % int(100.0 * (analyzeRangeMax - analyzeRangeMin)))
	//	/ 100.0;
	analyzeRange = 1.3;
}


bool ObstacleDetection::isRangeSmaller(double distance, std::vector<float> ranges) {
	for(int i = 0; i < ranges.size(); i++)
		if(ranges[i] < distance) {
			return true;
		}
	return false;
}

void ObstacleDetection::analyzeLaserScan(geometry_msgs::Point& position, double orientation,
                                         const sensor_msgs::LaserScan& msg) {

	std::vector<geometry_msgs::Point> points = getPointList(position, orientation, msg);
	std::vector<std::vector<int>> obs = detectObstacle(points);

	if(obstacleDetected) {
		//TODO: What to do when there is an unanticipated obstacle detected? The obstacle can be another agent.

		//send local path request if obstacles differ from previous
		if(motionPlanner->hasPath()) {
			std::vector<geometry_msgs::Point> obstacles;
			for(int i = 0; i < obs.size(); i++) {
				//only hand over obstacles border points
				obstacles.push_back(points[obs[i][0]]);
				obstacles.push_back(points[obs[i][1]]);
			}
			// TODO: If obstacles are not known, then they are other agents. Go to dealWithObstacles for your strategies
			dealWithObstacles(obstacles);
		}

		//continue driving if no obstacle has been sensed
	} else if(motionPlanner->hasPath()) {
		motionPlanner->start();
	}
}

/*
 * if some point is irrelevant then p.z is set to -1
 */
std::vector<geometry_msgs::Point> ObstacleDetection::getPointList(geometry_msgs::Point position,
                                                                  double orientation,
                                                                  const sensor_msgs::LaserScan& msg) {
	std::vector<geometry_msgs::Point> points;
	double min_laser_orientation = orientation + (msg.angle_min) / 2.0;

	//calculate position of the laser
	double laser_rel_x = 0.2;
	position.x = position.x + cos(2 * orientation) * laser_rel_x;
	position.y = position.y + sin(2 * orientation) * laser_rel_x;
	for(int i = 0; i < msg.ranges.size(); i++) {
		geometry_msgs::Point p;
		if(msg.ranges[i] < analyzeRange) {
			double real_orientation = min_laser_orientation + (i * msg.angle_increment) / 2.0;
			if(real_orientation > PI / 2.0) {
				real_orientation = real_orientation - PI;
			} else if(real_orientation < -PI / 2.0) {
				real_orientation = real_orientation + PI;
			}
			real_orientation *= 2;
			p.x = position.x + cos(real_orientation) * msg.ranges[i];
			p.y = position.y + sin(real_orientation) * msg.ranges[i];
		} else {
			p.z = -1;
		}
		points.push_back(p);
	}
	return points;
}

std::vector<std::vector<int>> ObstacleDetection::detectObstacle(std::vector<geometry_msgs::Point>& points) {
	bool detected = false;
	bool is_obstacle = false;
	int min_obstacle_index_width = 3;
	bool end_of_obstacle = false;

	std::vector<std::vector<int>> obs;
	for(int i = 0; i < points.size(); i++) {
		if(points[i].z == -1) {
			if(is_obstacle) {
				end_of_obstacle = true;
			}
		} else {
			bool unexpected_point = true;
			int x_index = points[i].x * 10;
			int y_index = points[i].y * 10;
			int range = 1;
			for(int j = -range - 1; j <= range; j++) {
				for(int k = -range - 1; k <= range; k++) {
					int a = x_index + j;
					int b = y_index + k;
					if(a >= 0 && b >= 0 &&
					   a < occupancyGrid[0].size() && b < occupancyGrid.size()) {
						if(occupancyGrid[b][a] == 100) {
							j = range + 1;
							k = range + 1;
							unexpected_point = false;
						}
					}
				}
			}

			if(!unexpected_point) {
				if(is_obstacle) {
					end_of_obstacle = true;
				}
			} else {
				if(!is_obstacle) {
					is_obstacle = true;
					std::vector<int> o = {i, i};
					obs.push_back(o);
				} else {
					obs.back()[1] = i;
				}

				if(i == points.size() - 1) {
					end_of_obstacle = true;
				}
			}
		}

		if(is_obstacle && end_of_obstacle) {
			is_obstacle = false;
			end_of_obstacle = false;
			if(obs.back()[1] - obs.back()[0] + 1 >= min_obstacle_index_width) {
				detected = true;
			}
		}
	}
	obstacleDetected = detected;
	return obs;
}

void ObstacleDetection::dealWithObstacles(std::vector<geometry_msgs::Point> obstacles) {
	//TODO: Fill this function with your strategies
	ROS_INFO("obstacleDetected::dealWithObstacles");

}
