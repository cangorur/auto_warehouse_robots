#include <agent/ChargingManagement.h>
#include "agent/Agent.h"

bool sortByDuration(const Path &lhs, const Path &rhs) { return lhs.getDuration() < rhs.getDuration(); }



ChargingManagement::ChargingManagement(Agent* a, auto_smart_factory::WarehouseConfiguration warehouse_configuration, Map* m) {

	agent = a;
	map = m;
	agentID = agent->getAgentID();
	agentBatt = agent->getAgentBattery();
	warehouseConfig = warehouse_configuration;
	ROS_INFO("[ChargingManagement] Started, agent ID: [%s], agent Batt: [%f] ", agentID.c_str(), agentBatt);
	getAllChargingStations();
}

ChargingManagement::~ChargingManagement() {
}


bool ChargingManagement::isEnergyAvailable(double energy){

	agentBatt = agent->getAgentBattery();
	energyAfterTask =  agentBatt - energy;
	if (energyAfterTask > criticalMinimum ){
        return true;
    }
    return false;
}


double ChargingManagement::getDischargedBattery(){
	agentBatt = agent->getAgentBattery();
	return (100-agentBatt);
}


void ChargingManagement::getAllChargingStations(){

	//Iterate through all the trays and separate out charging trays
	for(int i = 0; i < warehouseConfig.trays.size(); i++) {
		if(warehouseConfig.trays[i].type.compare("charging station")==0) {
			charging_trays.push_back(warehouseConfig.trays[i]);
		}
	}

	ROS_INFO("[Charging Management]:Found (%d) Charging Stations !",(unsigned int) charging_trays.size());

	for(int i= 0; i<charging_trays.size(); i++){
		ROS_INFO("[Charging Management]:Charging Station ID :%d, Position: x = %f, y = %f !",charging_trays[i].id, charging_trays[i].x, charging_trays[i].y );
	}
}

Path ChargingManagement::getPathToNearestChargingStation(OrientedPoint start, double startingTime){

	//Vector for all paths found
	std::vector<Path> paths;

	//Find paths for all possible charging stations
	for(int i= 0; i<charging_trays.size(); i++){
		paths.push_back(map->getThetaStarPath(start,charging_trays[i], startingTime));
	}

    //Sort paths according to their duration, lower is better
	std::sort(paths.begin(), paths.end(), sortByDuration);

	//Lists all paths
    for(int i= 0; i< paths.size(); i++){
		ROS_INFO("[Charging Management]:Path to Charging Station Rank :%d, Duration:%f !",i, paths[i].getDuration());
	}

    return paths[0];
}


double ChargingManagement::getScoreMultiplier(float cumulatedEnergyConsumption){

	agentBatt = agent->getAgentBattery();
	energyAfterTask =  agentBatt - cumulatedEnergyConsumption; 	//Expected energy in agent after all the tasks

	if(energyAfterTask > upperThreshold){
	    return 1;
	}

	else if (energyAfterTask > lowerThreshold){
		//Calculate factor using the quadratic function : y = (-0.01)x^2+2 * (lowerThreshold/100)* x- lowerThreshold

		double quadraticFraction =  (2 + 0.01*lowerThreshold) * energyAfterTask - pow(energyAfterTask,2) * 0.01 - lowerThreshold;
		return quadraticFraction/100;
	}
	return energyAfterTask/100;
}


