#include "evaluation/Evaluator.h"

Evaluator::Evaluator() {
	ros::NodeHandle n("~");
	numberTransportationTasks = 0;
	fileName = std::string(std::getenv("HOME")) + fileName;
	evalSub = n.subscribe("/task_evaluation", 1, &Evaluator::evaluationCallback, this);
	start = ros::Time::now().toSec();
}

void Evaluator::evaluationCallback(const auto_smart_factory::TaskEvaluation& evalMsg) {
	evalMessages.push_back(evalMsg);
	if(evalMsg.task_type == "transportation") {
		numberTransportationTasks++;
		if(numberTransportationTasks == writeNumber) {
			ROS_WARN("[Evaluator] Writing to %s", fileName.c_str());
			writeEvaluations();
			ROS_FATAL("[Evaluator] Written to File");
		}
	}
}

void Evaluator::writeEvaluations() {
	FILE* f;
	f = fopen(fileName.c_str(), "w");
	ROS_ASSERT_MSG(f != nullptr,"[Evaluator] could not open file");
	for(auto_smart_factory::TaskEvaluation msg : evalMessages) {
		writeEvaluationData(f, msg);
	}
	fclose(f);
}

void Evaluator::writeEvaluationData(FILE* f, auto_smart_factory::TaskEvaluation& msg) {
	// compute all values for a task
	char buffer[128];
	double total = msg.finishedAt - msg.assignedAt;
	double execution = msg.finishedAt - msg.startedAt;
	double waiting = msg.startedAt - msg.assignedAt;
	double assigned = msg.assignedAt - start;
	if(msg.task_type == "transportation") {
		double driveToPickUp = msg.startedPickUpAt - msg.startedAt;
		double pickup = msg.finishedPickUpAt - msg.startedPickUpAt;
		double driveToDropOff = msg.startedDropOffAt - msg.finishedPickUpAt;
		double dropOff = msg.finishedAt - msg.startedDropOffAt;
		// taskType; robot id; id; time since start of simultion till assignment; total; execution; waiting; DriveToPickup; Pickup; DriveToDropOff; DropOff; Battery level at end
		sprintf(buffer, "%s;%s;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n", msg.task_type.c_str(), msg.robot_id.c_str(), 
				msg.request_id, assigned, total, execution, waiting, driveToPickUp, pickup, driveToDropOff, dropOff, msg.endBatteryLevel);
	} else if(msg.task_type == "charging") {
		double driveToCharging = msg.arrived_at - msg.startedAt;
		double charging = msg.finishedAt - msg.arrived_at;
		// taskType; robot_id; -1; time since start of simultion till assignment; total; execution; waiting; driveToCharging, charging 
		sprintf(buffer, "%s;%s;-1;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\n", msg.task_type.c_str(), msg.robot_id.c_str(), 
				assigned, total, execution, waiting, driveToCharging, charging);
	}
	fputs(buffer, f);
}