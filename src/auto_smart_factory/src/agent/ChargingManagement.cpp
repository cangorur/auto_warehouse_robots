#include <agent/ChargingManagement.h>
#include "agent/Agent.h"


ChargingManagement::ChargingManagement(Agent* a) {

	agent = a;
	agentID = agent->getAgentID();
	agentBatt = agent->getAgentBattery();
	//ROS_INFO("[ChargingManagement] Started, agent ID: [%s], agent Batt: [%f] ", agentID.c_str(), agentBatt);

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


