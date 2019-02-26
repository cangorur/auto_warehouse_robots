#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include "ros/ros.h"
#include "auto_smart_factory/TaskEvaluation.h"

class Evaluator {
public:
	Evaluator();

	virtual ~Evaluator() = default;

	void evaluationCallback(const auto_smart_factory::TaskEvaluation& evalMsg);
	
private:
	void writeEvaluations();

	void writeEvaluationData(FILE* f, auto_smart_factory::TaskEvaluation& evalMsg);

	const unsigned int writeNumber = 40;

	unsigned int numberTransportationTasks;

	double start;

	ros::Subscriber evalSub;

	std::vector<auto_smart_factory::TaskEvaluation> evalMessages;

	std::string fileName = "/.ros/log/evaluation.csv";
};


#endif