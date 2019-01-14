#include <agent/ChargingManagement.h>
#include "agent/Agent.h"


ChargingManagement::ChargingManagement(Agent* a) {

	agent = a;
	agentID = agent->getAgentID();
	agentBatt = agent->getAgentBattery();
	ROS_INFO("[ChargingManagement] Started, agent ID: [%d], agent Batt: [%f] ", agentID, agentBatt);

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


float ChargingManagement::getScoreMultiplier(float cumulatedEnergyConsumption){

	agentBatt = agent->getAgentBattery();
	energyAfterTask =  agentBatt - cumulatedEnergyConsumption;
	if(energyAfterTask > upperThreshhold){
	    return 1;
	}
	return energyAfterTask/100;

}


