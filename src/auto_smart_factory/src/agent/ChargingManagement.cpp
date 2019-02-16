#include "agent/ChargingManagement.h"
#include "agent/Agent.h"

ChargingManagement::ChargingManagement(Agent* a, auto_smart_factory::WarehouseConfiguration warehouse_configuration, auto_smart_factory::RobotConfiguration robot_configuration, Map* m) {
	agent = a;
	map = m;
	agentID = agent->getAgentID();
	warehouseConfig = warehouse_configuration;
	robotConfig = robot_configuration;

	dischargingRate = robotConfig.discharging_rate;
	chargingRate = robotConfig.charging_rate;
	motorDrainingRate = robotConfig.motor_draining_rate;

	ROS_INFO("[Charging Management %d] Battery: %f, Charging Rate: %f, Discharging Rate: %f, Motor Draining Rate: %f", agent->getAgentIdInt(), agent->getAgentBattery(), chargingRate, dischargingRate, motorDrainingRate);

	getAllChargingStations();
}

void ChargingManagement::getAllChargingStations(){
	//Iterate through all the trays and separate out charging trays
	for(auto& tray : warehouseConfig.trays) {
		if(tray.type == "charging station") {
			charging_trays.push_back(tray);
		}
	}

	//ROS_INFO("[Charging Management]:Found (%d) Charging Stations !",(unsigned int) charging_trays.size());
}

std::pair<Path, uint32_t> ChargingManagement::getPathToNearestChargingStation(OrientedPoint start, double startingTime) {
	u_int32_t nearestStationId = 0;
	Path shortestPath;
	double nearestStationDuration = std::numeric_limits<double>::max();
	
	//Find paths for all possible charging stations
	for(auto& charging_tray : charging_trays) {
		if(map->isPointTargetOfAnotherRobot(charging_tray)) {
			continue;
		}		
		
		Path path = map->getThetaStarPath(start, charging_tray, startingTime, ChargingTask::getChargingTime());
		if(path.isValid() && path.getDuration() < nearestStationDuration) {
			nearestStationDuration = path.getDuration();
			
			shortestPath = path;
			nearestStationId = charging_tray.id;
		}
	}
	
    return std::make_pair(shortestPath, nearestStationId);
}

double ChargingManagement::getScoreMultiplierForBatteryLevel(double batteryLevel) {
	if(batteryLevel >= upperThreshold) {
	    return 1.0;
	} else if (batteryLevel >= lowerThreshold) {
		//Calculate factor using the quadratic function : y = (-0.01)x^2+2 * (lowerThreshold/100)* x- lowerThreshold
		double quadraticFraction =  (2.f + 0.01f * lowerThreshold) * batteryLevel - pow(batteryLevel, 2) * 0.01f - lowerThreshold;
		return quadraticFraction / 100.f;
	} else if(batteryLevel >= criticalMinimum) {
		// Linear function
		return batteryLevel / 100.f;
	}
	return 0.0;
}

double ChargingManagement::getChargingTime(double consumptionTillCS){
	return ((agent->getAgentBattery() - consumptionTillCS) /chargingRate) ;
}

bool ChargingManagement::isChargingAppropriate() {
	return (agent->getAgentBattery() <= chargingAppropriateLevel);
}

bool ChargingManagement::isCharged() {
	bool charged = agent->getAgentBattery() >= 99.9f;
	if(charged) {
		ROS_INFO("[Agent %d] Finished charging with %f%%", agent->getAgentIdInt(), agent->getAgentBattery());
	}
	return charged;
}

double ChargingManagement::getCurrentBattery() {
	return agent->getAgentBattery();
}

bool ChargingManagement::isConsumptionPossible(double consumption) {
	return agent->getAgentBattery() - consumption - estimatedBatteryConsumptionToNearestChargingStation > criticalMinimum;
}

bool ChargingManagement::isConsumptionPossible(double agentBatteryLevel, double consumption) {
	return agentBatteryLevel - consumption - estimatedBatteryConsumptionToNearestChargingStation > criticalMinimum;
}
