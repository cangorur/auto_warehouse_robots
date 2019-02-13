#include "evaluation/Evaluator.h"

Evaluator::Evaluator() {
	ros::NodeHandle n("~");
	numberTransportationTasks = 0;
	fileName = std::string(std::getenv("HOME")) + fileName;
	evalSub = n.subscribe("/task_evaluation", 1, &Evaluator::evaluationCallback, this);
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
	if(msg.task_type == "transportation") {
		double driveToPickUp = msg.startedPickUpAt - msg.startedAt;
		double pickup = msg.finishedPickUpAt - msg.startedPickUpAt;
		double driveToDropOff = msg.startedDropOffAt - msg.finishedPickUpAt;
		double dropOff = msg.finishedAt - msg.startedDropOffAt;
		// taskType; robot id; id; total; execution; waiting; DriveToPickup; Pickup; DriveToDropOff; DropOff ((6+2)+2+7*6) = 52Bytes
		sprintf(buffer, "%s;%s;%d;%4.2f;%4.2f;%4.2f;%4.2f;%4.2f;%4.2f;%4.2f\n", msg.task_type.c_str(), msg.robot_id.c_str(), 
				msg.request_id, total, execution, waiting, driveToPickUp, pickup, driveToDropOff, dropOff);
	} else if(msg.task_type == "charging") {
		double driveToCharging = msg.arrived_at - msg.startedAt;
		double charging = msg.finishedAt - msg.arrived_at;
		// taskType; robot_id; total; execution; waiting; driveToCharging, charging 
		sprintf(buffer, "%s;%s;%4.2f;%4.2f;%4.2f;%4.2f;%4.2f\n", msg.task_type.c_str(), msg.robot_id.c_str(), 
				total, execution, waiting, driveToCharging, charging);
	}
	fputs(buffer, f);
}
