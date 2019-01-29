#include <agent/ChargingManagement.h>
#include "agent/Agent.h"


ChargingManagement::ChargingManagement(Agent* a, auto_smart_factory::WarehouseConfiguration warehouse_configuration) {

	agent = a;
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


void ChargingManagement::getAllChargingStations(){

	//Iterate through all the trays and separate out charging trays
	for(int i = 0; i < warehouseConfig.trays.size(); i++) {
		if(warehouseConfig.trays[i].type.compare("charging station")) {
			charging_trays.push_back(warehouseConfig.trays[i]);
		}
	}

	//Fill charging stations vector

//	for(int i= 0; i<charging_trays.size(); i++){

		//ID of charging station, same as iterator for now
//		charging_stations[i].id = i;

		//Tray corresponding to charging station
//		charging_stations[i].Tray = charging_trays[i];

		//All stations unoccupied
//		charging_stations[i].occupancy = false;

//		charging_stations[i].robot = {};

//		charging_stations[i].rate = 0;


//	}
}


bool ChargingManagement :: isChargingStationAvailable(uint8_t searchid){

	for(int i= 0; i<charging_stations.size(); i++){
		if(charging_stations[i].id == searchid){
			return charging_stations[i].occupancy;
		}
		else {
			ROS_ERROR("[Charging Management]: Searched for Non Existent Charging Station (%d) !",searchid);
			return false;
		}
	}
}

bool ChargingManagement :: reserveChargingStation(uint8_t reserveid, auto_smart_factory::Robot associated_robot){

	for(int i= 0; i<charging_stations.size(); i++){
		if(charging_stations[i].id == reserveid){
			charging_stations[i].occupancy = true;
			charging_stations[i].robot = associated_robot;
			return true;
		}
		else {
			ROS_ERROR("[Charging Management]: Tried to reserve a Non Existent Charging Station (%d) !",reserveid);
			return false;
		}
	}
}


bool ChargingManagement :: unreserveChargingStation(uint8_t reserveid){

	for(int i= 0; i<charging_stations.size(); i++){
		if(charging_stations[i].id == reserveid){
			charging_stations[i].occupancy = false;
			charging_stations[i].robot = {};
			return true;
		}
		else {
			ROS_ERROR("[Charging Management]: Tried to unreserve a Non Existent Charging Station (%d) !",reserveid);
			return false;
		}
	}
}


float ChargingManagement::getScoreMultiplier(float cumulatedEnergyConsumption){

	agentBatt = agent->getAgentBattery();
	energyAfterTask =  agentBatt - cumulatedEnergyConsumption; 	//Expected energy in agent after all the tasks

	if(energyAfterTask > upperThreshold){
	    return 1;
	}

	else if (energyAfterTask > lowerThreshold){
		//Calculate factor using the quadratic function : y = (-0.01)x^2+2 * (lowerThreshold/100)* x- lowerThreshold

		float quadraticFraction =  (2 + 0.01*lowerThreshold) * energyAfterTask - pow(energyAfterTask,2) * 0.01 - lowerThreshold;
		return quadraticFraction/100;
	}
	return energyAfterTask/100;
}


