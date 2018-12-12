#include "agent/PidController.h"
#include "Math.h"

PidController::PidController(ros::Publisher* pub, double posTolerance, double angleTolerance, double maxSpeed, double maxAngleSpeed) 
    :
    pubVelocity(pub),
    posTolerance(posTolerance),
    angleTolerance(angleTolerance),
    maxSpeed(maxSpeed),
    maxAngleSpeed(maxAngleSpeed),
    targetDistance(0.0),
    targetAngle(0.0),
    iterations(0),
    sumDistance(0.0),
    sumAngle(0.0),
    start(new Position()),
    last(new Position()) {
}

void PidController::reset() {
    iterations = 0;
    sumDistance = 0.0;
    sumAngle = 0.0;
    delete start;
    delete last;
    start = new Position();
    last = new Position();
}

void PidController::setTargetValue(double distance, double angle) {
    targetDistance = distance;
    targetAngle = angle;
}

void PidController::setTargetPoint(Point target, Position position) {
    float distToTarget = Math::getDistance(Point(position.x, position.y), target);
	float rotationToTarget = getRotationToTarget(Point(position.x, position.y), target, position.o);

    setTargetValue(distToTarget, rotationToTarget);
}

float PidController::getRotationToTarget(Point currentPosition, Point targetPosition, double orientation) {
	double direction = std::atan2(targetPosition.y - currentPosition.y, targetPosition.x - currentPosition.x);

	return static_cast<float>(Math::getAngleDifferenceInRad(orientation, direction));
}


void PidController::publishVelocity(double speed, double angle) {
    geometry_msgs::Twist msg;
    
    msg.linear.x = speed;
    msg.angular.z = angle;

    pubVelocity->publish(msg);
}

bool PidController::update(Position* current) {
    double newAngle = 0.0;
    double newSpeed = 0.0;

    if (targetReached(current))
    {
        ROS_INFO("GOAL ACHIEVED");
        publishVelocity(0.0, 0.0);
        return true;
    }

    if (iterations == 0) {
        *start = *current;
        *last = *current;
    }
    iterations++;

    //Calculation of action intervention.
    if (fabs(targetDistance) > posTolerance) {
        newSpeed = calculatePSD(current, start->getDistance(current) * copysign(1.0, targetDistance), start->getDistance(last) * copysign(1.0, targetDistance), targetDistance, F_KP, F_KD, F_KI, &sumDistance);
    }

    if (current->o - last->o < -M_PI) {
        current->o += 2*M_PI;
    } else if (current->o - last->o > M_PI) {
        current->o -= 2*M_PI;
    }

    newAngle = calculatePSD(current, current->o - start->o, last->o - start->o, targetAngle, R_KP, R_KD, R_KI, &sumAngle);

    *last = *current;

    // publish velocity message
    publishVelocity(fmin(maxSpeed, newSpeed), fmin(maxAngleSpeed, newAngle));

    return false;
}

bool PidController::targetReached(Position* current) {
    double distance;
    distance = start->getDistance(current)*copysign(1.0, targetDistance);

    if (fabs(distance-targetDistance) > posTolerance) {
        return false;
    }

    if (fabs(targetAngle - (current->o - start->o)) > angleTolerance &
                fabs(targetAngle - (current->o - start->o) + 2*M_PI) > angleTolerance &
                fabs(targetAngle - (current->o - start->o) - 2*M_PI) > angleTolerance) {
        return false;
    }

    return true;
}

double PidController::calculatePSD(Position* current, double currentValue, double lastValue, double referenceValue, double kP, double kD, double kS, double* sum) {
    double speed = 0;
    double error = referenceValue - currentValue;
    double previousError = referenceValue - lastValue;
    double dt = current->t.toSec() - last->t.toSec();
    double derivative = (error - previousError)/dt;
    *sum = *sum + error*dt;
    //speed = kP*error + kD*derivative + kS*(*sum);
    speed = kP*error + kS*(*sum);

    return speed;
}

PidController::~PidController() {
    delete start;
    delete last;
}