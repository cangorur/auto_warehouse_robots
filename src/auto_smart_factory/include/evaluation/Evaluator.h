#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include "ros/ros.h"
#include "auto_smart_factory/TaskEvaluation.h"

class Evaluator {
public:
	/**
	 * Constructor for evaluation data collector
	 */ 
	Evaluator();

	/**
	 * Destructor
	 */
	virtual ~Evaluator() = default;

	/**
	 * Callback for evaluation Messages from tasks
	 */
	void evaluationCallback(const auto_smart_factory::TaskEvaluation& evalMsg);
	
private:
	/**
	 * Write all collected evalaution messages to file
	 */
	void writeEvaluations();

	/**
	 * write single evaluation message to specific file
	 * @param f, file to be written to
	 * @param evalMsg, evaluation message to be written
	 * Format is as follows:
	 * 		transportation tasks:	taskType; robot id; id; time since start of simultion till assignment; total; execution; waiting; DriveToPickup; Pickup; DriveToDropOff; DropOff; Battery level at end
	 * 		charging tasks: 		taskType; robot_id; -1; time since start of simultion till assignment; total; execution; waiting; driveToCharging, charging
	 */
	void writeEvaluationData(FILE* f, auto_smart_factory::TaskEvaluation& evalMsg);

	// Number of received evaluation messages for transportation tasks at which the 
	// Evaluator writes to file  
	const unsigned int writeNumber = 40;

	// The number of received evaluation messages for transportation tasks
	unsigned int numberTransportationTasks;

	// The timestamp at which the simulation started
	double start;

	// the subscriber for task evaluation messages
	ros::Subscriber evalSub;

	// vector containing the saved evaluation messages 
	std::vector<auto_smart_factory::TaskEvaluation> evalMessages;

	// the file to be written to
	// Note: this string will be appended to the path to the home directory 
	std::string fileName = "/.ros/log/evaluation.csv";
};


#endif