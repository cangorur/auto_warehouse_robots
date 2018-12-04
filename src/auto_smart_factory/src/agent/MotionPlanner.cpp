#include <agent/MotionPlanner.h>
#include <agent/Agent.h>

MotionPlanner::MotionPlanner(Agent* a, auto_smart_factory::RobotConfiguration robot_config,
                             ros::Publisher* motion_pub) {
	robotConfig = robot_config;
	drivingMinSpeed = robotConfig.min_linear_vel;
	drivingMaxSpeed = robotConfig.max_linear_vel;
	turningMaxSpeed = robotConfig.max_angular_vel;
	motionPub = motion_pub;
	agent = a;
	agentID = agent->getAgentID();
	std::stringstream ss;
	ss << "/motion_planner_" << agentID << "/activate_tests";
	test_enable_paramStr = ss.str();
	ros::param::set(test_enable_paramStr, tests_enabled); // false by default
}

MotionPlanner::~MotionPlanner() {
}

void MotionPlanner::update(geometry_msgs::Point position, double orientation) {
	bool prev_test_enable = tests_enabled;
	ros::param::get(test_enable_paramStr, tests_enabled);
	if(enabled || tests_enabled) { // if "enabled" by the operation or "test_enable" is triggered
		if(tests_enabled) { // if test enable, the robot will only be controlled with manual commands
			if(!prev_test_enable) { // if tests are just enabled, first stop the prev motion command
				stop();
			}
			start();
		} else if(!done && !standStill) {
			done = driveCurrentPath(position, orientation);
		} else {
			stop();
		}
	}
}

void MotionPlanner::newPath(geometry_msgs::Point start_position, std::vector<geometry_msgs::Point> new_path,
                            geometry_msgs::Point end_direction_point, bool drive_backwards) {
	startPosition = start_position;
	driveBackwards = drive_backwards;
	path = new_path;
	smoothPath(path);
	endDirectionPoint = end_direction_point;
	currentTargetIndex = 0;
	turnDirection = 0;
	turningStart = true;
	numberTurns = 0;
	turning = true;
	done = false;
}

void MotionPlanner::smoothPath(std::vector<geometry_msgs::Point>& path, double weight_data, double weight_smooth,
                               double tolerance) {

	std::vector<geometry_msgs::Point> newPath = path;        //Deep copy

	for(int k = 0; k < newPath.size(); ++k) {
		if(k != 0 and k != path.size() - 1) {
			double force_org_x, force_org_y, force_dyn_x, force_dyn_y, force_tot_x = 10, force_tot_y = 10;
			while((pow(force_tot_x, 2) + pow(force_tot_y, 2)) >= pow(tolerance, 2)) {
				force_org_x = weight_data * (path[k].x - newPath[k].x);
				force_dyn_x = weight_smooth * (newPath[k - 1].x + newPath[k + 1].x - (2 * newPath[k].x));   //TODO 2*
				force_tot_x = force_org_x + force_dyn_x;
				newPath[k].x += force_tot_x;

				force_org_y = weight_data * (path[k].y - newPath[k].y);
				force_dyn_y = weight_smooth * (newPath[k - 1].y + newPath[k + 1].y - (2 * newPath[k].y));   //TODO 2*
				force_tot_y = force_org_y + force_dyn_y;
				newPath[k].y += force_tot_y;
			}
		}
	}
	path = newPath;
}

bool MotionPlanner::comparePaths(std::vector<geometry_msgs::Point> new_path) {
	int j = path.size() - 1;
	for(int i = new_path.size() - 1; i >= 0; i--) {
		if(j < 0 || new_path[i].x != path[j].x || new_path[i].y != path[j].y) {
			return false;
		}
		j--;
	}
	if(j > currentTargetIndex) {
		return false;
	}
	return true;
}

void MotionPlanner::enable(bool enable) {
	this->enabled = enable;
}

bool MotionPlanner::isEnabled() {
	return this->enabled;
}

bool MotionPlanner::isTestsEnabled() { // adjusted through ros param
	return this->tests_enabled;
}

void MotionPlanner::start() {
	this->standStill = false;
}

void MotionPlanner::stop() {
	this->standStill = true;
	geometry_msgs::Twist motion;
	this->motionPub->publish(motion);
}

bool MotionPlanner::isDone() {
	return done;
}

bool MotionPlanner::isTurning() {
	return turning;
}

bool MotionPlanner::isTurningLeft() {
	return turnsLeft;
}

double MotionPlanner::getTurnDirection() {
	return turnDirection;
}

void MotionPlanner::changeTurningDirection() {
	turningStart = false;
	turnsLeft = !turnsLeft;
	numberTurns++;
}

int MotionPlanner::getNumberTurns() {
	return numberTurns;
}

bool MotionPlanner::hasPath() {
	return path.size() > 0;
}

std::vector<geometry_msgs::Point> MotionPlanner::getRemainingPath() {
	std::vector<geometry_msgs::Point> p;
	for(int i = currentTargetIndex; i < path.size(); i++)
		p.push_back(path[i]);
	return p;
}

bool MotionPlanner::isDrivingBackwards() {
	return driveBackwards;
}

bool MotionPlanner::hasDifferentGoal(std::vector<geometry_msgs::Point> new_path) {
	//prevent new path from having a different target
	if(getGoalPosition().x != path[path.size() - 1].x ||
	   getGoalPosition().y != path[path.size() - 1].y) {
		return true;
	}
	return false;
}

geometry_msgs::Point MotionPlanner::getLastPosition() {
	if(currentTargetIndex <= 0) {
		return startPosition;
	} else if(currentTargetIndex >= path.size()) {
		return path[path.size() - 1];
	} else {
		return path[currentTargetIndex - 1];
	}
}

geometry_msgs::Point MotionPlanner::getNextPosition() {
	if(currentTargetIndex <= 0) {
		return path[0];
	} else if(currentTargetIndex >= path.size()) {
		return path[path.size() - 1];
	} else {
		return path[currentTargetIndex];
	}
}

geometry_msgs::Point MotionPlanner::getGoalPosition() {
	return path[path.size() - 1];
}

geometry_msgs::Point MotionPlanner::getEndDirectionPoint() {
	return endDirectionPoint;
}

double MotionPlanner::getDriveForwardDistanceAccuracy() {
	return driveForwardDistanceAccuracy;
}

double MotionPlanner::getDriveBackwardsDistanceAccuracy() {
	return driveBackwardsDistanceAccuracy;
}

bool MotionPlanner::driveCurrentPath(geometry_msgs::Point position, double orientation) {
	geometry_msgs::Twist motion;
	if(currentTargetIndex < path.size() || turning) {
		double p_x = position.x;
		double p_y = position.y;
		orientation = orientation / (PI / 2.0);
		geometry_msgs::Point t;
		if(currentTargetIndex < path.size()) {
			t = path[currentTargetIndex];
		} else {
			//to handle endorientation the right way
			if(driveBackwards) {
				driveBackwards = false;
			}
			t = endDirectionPoint;
		}


		if(turning) {
			motion = turnOnSpot(p_x, p_y, t.x, t.y, orientation);
		}
		if(!turning && currentTargetIndex < path.size()) {
			motion = driveStraight(p_x, p_y, t.x, t.y, orientation);
		}

		//motion.linear.x = 0.7;
		//standStill = false;

		if(standStill) {
			stop();
		} else {
			motionPub->publish(motion);
		}
		return false;
	} else {
		stop();
		return true;
	}
}

geometry_msgs::Twist MotionPlanner::driveStraight(double p_x, double p_y, double t_x, double t_y,
                                                  double orientation) {

	geometry_msgs::Twist motion;
	double distance = getDistance(p_x, p_y, t_x, t_y);

	double d_accuracy = driveForwardDistanceAccuracy;
	//bigger backwards accuracy
	//tried to work around failing to reach back driving goal
	if(driveBackwards) {
		d_accuracy = driveBackwardsDistanceAccuracy;
	}

	if(distance > d_accuracy) {
		motion.linear.x = 1.0 * distance;
		motion.linear.x = scaleSpeed(motion.linear.x, drivingMaxSpeed, drivingMinSpeed);

		if(driveBackwards) {
			motion.linear.x *= -1.0;
		}


		double direction = getDirection(p_x, p_y, t_x, t_y);
		double orientation_diff = getOrientationDiff(orientation, direction);

		if(fabs(orientation_diff) > steeringAccuracy) {
			motion.angular.z = 17 * orientation_diff * fabs(motion.linear.x)
			                   / drivingMaxSpeed;
			motion.angular.z = scaleSpeed(motion.angular.z, turningMaxSpeed, 0);
			if(driveBackwards) {
				motion.angular.z *= -1.0;
			}
		}
	} else {
		nextWayPoint();
	}

	return motion;
}

void MotionPlanner::nextWayPoint() {
	currentTargetIndex++;
	numberTurns = 0;
	turning = true;
}

geometry_msgs::Twist MotionPlanner::turnOnSpot(double p_x, double p_y, double t_x, double t_y,
                                               double orientation) {
	//turnOnSpotAccuracy = 0.6;
	bool stop = false;
	geometry_msgs::Twist motion;
	if(p_x == t_x && p_y == t_y) {
		stop = true;
	} else {
		double direction;
		if(driveBackwards) {
			direction = getDirection(t_x, t_y, p_x, p_y);
		} else {
			direction = getDirection(p_x, p_y, t_x, t_y);
		}

		double orientation_diff = getOrientationDiff(orientation, direction);

		if(fabs(orientation_diff) > turnOnSpotAccuracy) {
			double speed = 0;
			if(turningStart) {
				speed = orientation_diff;
				turnsLeft = speed >= 0;
			} else {
				if(turnsLeft) {
					if(orientation_diff > 0) {
						speed = orientation_diff;
					} else {
						speed = 2 + orientation_diff;
					}
				} else {
					if(orientation_diff > 0) {
						speed = -2 + orientation_diff;
					} else {
						speed = orientation_diff;
					}
				}
			}
			speed = 10 * speed;
			speed = scaleSpeed(speed, turningMaxSpeed, turningMinSpeed);
			motion.angular.z = speed;
			turnDirection = speed;
		} else {
			stop = true;
		}
	}

	if(stop) {
		turning = false;
		turningStart = true;
		turnDirection = 0;
	}
	return motion;
}

double MotionPlanner::scaleSpeed(double speed, double max, double min) {
	int p = (speed >= 0) ? 1 : -1;
	if(p * speed > max) {
		return p * max;
	} else if(p * speed < min) {
		return p * min;
	} else {
		return speed;
	}
}

double MotionPlanner::getDirection(double p_x, double p_y, double t_x, double t_y) {
	return (std::atan2(t_y - p_y, t_x - p_x)) / PI;
}

double MotionPlanner::getDistance(double p1_x, double p1_y, double p2_x, double p2_y) {
	return sqrt(pow(p1_x - p2_x, 2.0) + pow(p1_y - p2_y, 2.0));
}

double MotionPlanner::getOrientationDiff(double orientation, double direction) {
	double diff;
	if(orientation >= 0 && direction >= 0) {
		diff = direction - orientation;
	} else if(orientation < 0 && direction < 0) {
		diff = direction - orientation;
	} else if(orientation >= 0 && direction < 0) {
		if(orientation - direction <= 1) {
			diff = direction - orientation;
		} else {
			diff = (1 - orientation) + (1 + direction);
		}
	} else if(orientation < 0 && direction >= 0) {
		if(direction - orientation <= 1) {
			diff = direction - orientation;
		} else {
			diff = direction - orientation - 2;
		}
	}
	return diff;
}

double MotionPlanner::getAngle(double orientation) {
	return (orientation >= 0) ? 180 * orientation : 360 + 180 * orientation;
}

void MotionPlanner::emgRetreat(float stop_interval, float retreat_interval) {
	geometry_msgs::Twist motion;
	motion.linear.x = -0.1;
	motion.linear.y = 0.0;
	motion.angular.x = 0.0;
	motion.angular.y = 0.0;
	motion.angular.z = 0.0;
	stop();
	ros::Duration(1 * stop_interval).sleep();
	motionPub->publish(motion);
	ROS_INFO("Retreating");
	ros::Duration(1 * retreat_interval).sleep();
	stop();
	start();
}
