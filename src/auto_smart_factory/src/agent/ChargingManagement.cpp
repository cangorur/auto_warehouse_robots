#include <agent/ChargingManagement.h>
#include "agent/Agent.h"


ChargingManagement::ChargingManagement(Agent* a) {

	agent = a;
	agentID = agent->getAgentID();
	agentBatt = agent->getAgentBattery();
	ROS_INFO("[ChargingManagement] Started, agent ID: [%s], agent Batt: [%f] ", agentID.c_str(), agentBatt);

}

ChargingManagement::~ChargingManagement() {
}


bool ChargingManagement::isEnergyAvailable(double energy){

	agentBatt = agent->getAgentBattery();
	energyAfterTask =  agentBatt - energy;
	if (energyAfterTask <= upperThreshohold && energyAfterTask > criticalMinimum ){
        return true;
    }
    return false;
}


float ChargingManagement::getScoreMultiplier(float cumulatedEnergyConsumption){

	//First, check if the charging can be done
	if(isEnergyAvailable(cumulatedEnergyConsumption)){
		return cumulatedEnergyConsumption / operatingBatt;
	}
	else return 0;
}


//void ChargingManagement::checkBattery(float batteryLevel){
//
//	if(batteryLevel <= 10){
//
//		//if batt <= 10, issue charging task immediately
//
//		ROS_ERROR("[Charging Management] Battery Critical!! [%f]", batteryLevel);
//
//	} else if(batteryLevel < 60 && batteryLevel > 10){
//
//		//if 10 < batt <60 && if idle, decide to charge or not based on distance to the charging station
//
//		ROS_INFO("[Charging Management] Battery discharging!! [%f]", batteryLevel);
//	}
//
//
//	// ID of the Charging station assigned to the agent
////	unsigned int chargingStationId;
//
//
//	// if batt => 60, do not issue charging task
//}
