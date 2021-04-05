#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include "ros/ros.h"
#include "auto_smart_factory/TaskEvaluation.h"
#include "auto_smart_factory/RobotHeartbeat.h"

class Evaluator {
public:
	/**
	 * Constructor for evaluation data collector
	 */ 
	Evaluator();

	/**
	 * Check if the node should write the evaluation to file
	 */
	void checkWrite();

	/**
	 * Destructor
	 */
	virtual ~Evaluator() = default;

	/**
	 * Callback for evaluation Messages from tasks
	 */
	void evaluationCallback(const auto_smart_factory::TaskEvaluation& evalMsg);

	/**
	 * Callback for robot heartbeat Messages
	 */
	void heartbeatCallback(const auto_smart_factory::RobotHeartbeat& heartMsg);
	
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
	 * 		transportation tasks:	taskType; robot_id; id; batteryConsumed; time since start of simultion till assignment; total; execution; waiting; DriveToPickup; Pickup; DriveToDropOff; DropOff; Battery level at end
	 * 		charging tasks: 		taskType; robot_id; -1; batteryConsumed; time since start of simultion till assignment; total; execution; waiting; driveToCharging, charging
	 */
	void writeEvaluationData(FILE* f, auto_smart_factory::TaskEvaluation& evalMsg);

	/**
	 * write single robot consumed battery
	 * Format ist:
	 * consumed_battery; robot_id; consumedBattery
	 */
	void writeConsumedBattery(FILE* f, std::map<std::string, std::pair<double, double> >::iterator it);

	// duration in minutes after which the evauator should write the evaluation data
	const double duration = 10;

	// has already written
	bool hasWritten = false;

	// flag for switching after writing
	bool writeTime = true;

	// Number of received evaluation messages for transportation tasks at which the 
	// Evaluator writes to file  
	const unsigned int writeNumber = 1;

	// The number of received evaluation messages for transportation tasks
	unsigned int numberTransportationTasks;

	// The timestamp at which the simulation started
	double start;

	// the subscriber for task evaluation messages
	ros::Subscriber evalSub;

	// the subscriber for the robot heartbeats
	ros::Subscriber heartSub;

	// map containing the id, and last battery level, consumed battery pairs
	// name,pair<last_battery_level, consumed_battery>
	std::map<std::string,std::pair<double, double> > robotConsumedEnergy;

	// vector containing the saved evaluation messages 
	std::vector<auto_smart_factory::TaskEvaluation> evalMessages;

	// the file to be written to
	// Note: this string will be appended to the path to the home directory 
	std::string fileName = "/.ros/log/evaluation.csv";
};


#endif