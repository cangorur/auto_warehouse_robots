#include "agent/PidController.h"

PidController::PidController(double targetValue, double proportionalGain, double integralGain, double derivativeGain) :
	targetValue(targetValue),
	proportionalGain(proportionalGain),
	integralGain(integralGain),
	derivativeGain(derivativeGain) {
}

double PidController::updateTargetValue(double targetValue) {
	this->targetValue = targetValue;
}

double PidController::calculate(double currentValue, double currentTime) {
	double currentError = targetValue - currentValue;
	double derivativeValue = 0.0;
	double integralValue = 0.0;

	if (this->previousTime >= 0)
	{
		double dt = currentTime - this->previousTime;
		derivativeValue = (currentError - this->previousError) / dt;
		integralValue = currentError * dt;
	}

	this->integralSum += integralValue;
	this->previousError = currentError;
	this->previousTime = currentTime;

	return proportionalGain * currentError + integralGain * integralSum + derivativeGain * derivativeValue;
}

void PidController::reset() {
	this->integralSum = 0.0;
	this->previousError = 0.0;
	this->previousTime = -1.0;
}