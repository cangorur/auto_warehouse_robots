#include "agent/ChargingManagement.h"
#include "agent/Agent.h"

ChargingManagement::ChargingManagement(Agent* a, auto_smart_factory::WarehouseConfiguration warehouse_configuration, Map* m) {
	agent = a;
	map = m;
	agentID = agent->getAgentID();
	warehouseConfig = warehouse_configuration;
	ROS_INFO("[ChargingManagement] Started, agent ID: [%s], agent Batt: [%f] ", agentID.c_str(), agent->getAgentBattery());
	getAllChargingStations();
}

void ChargingManagement::getAllChargingStations(){
	//Iterate through all the trays and separate out charging trays
	for(auto& tray : warehouseConfig.trays) {
		if(tray.type == "charging station") {
			charging_trays.push_back(tray);
		}
	}

	ROS_INFO("[Charging Management]:Found (%d) Charging Stations !",(unsigned int) charging_trays.size());
}

std::pair<Path, uint32_t> ChargingManagement::getPathToNearestChargingStation(OrientedPoint start, double startingTime) {
	u_int32_t nearestStationId = 0;
	Path shortestPath;
	double nearestStationDuration = std::numeric_limits<double>::max();
	
	//Find paths for all possible charging stations
	for(auto& charging_tray : charging_trays) {
		if(map->isPointTargetOfAnotherRobot(charging_tray)) {
			ROS_INFO("Skipping charging station because it is occupied");
			continue;
		}		
		
		Path path = map->getThetaStarPath(start, charging_tray, startingTime);
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
	    return 1;
	} else if (batteryLevel > lowerThreshold) {
		//Calculate factor using the quadratic function : y = (-0.01)x^2+2 * (lowerThreshold/100)* x- lowerThreshold
		double quadraticFraction =  (2 + 0.01f * lowerThreshold) * batteryLevel - pow(batteryLevel, 2) * 0.01f - lowerThreshold;
		return quadraticFraction / 100.f;
	}
	return batteryLevel / 100.f;
}

bool ChargingManagement::isChargingAppropriate() {
	return (agent->getAgentBattery() <= upperThreshold);
}

bool ChargingManagement::isCharged() {
	return agent->getAgentBattery() >= 99.5f;
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
