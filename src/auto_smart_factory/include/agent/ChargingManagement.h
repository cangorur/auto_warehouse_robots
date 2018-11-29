#ifndef AGENT_CHARGINGMANAGEMENT_H_
#define AGENT_CHARGINGMANAGEMENT_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include "auto_smart_factory/InitChargingManagement.h"
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/RobotHeartbeat.h>
#include "auto_smart_factory/RegisterAgentCharging.h"
#include "auto_smart_factory/AssignChargingTask.h"
#include "auto_smart_factory/GetFreeChargingStations.h"

/**
 * The charging management component manages all charging stations, provides information about free
 * charging stations and offers a service to reserve charging stations.
 */
class ChargingManagement{
public:
	/**
	 * Default constructor.
	 * Sets up the initialize service.
	 */
	ChargingManagement();
	virtual ~ChargingManagement();

};

#endif /* AGENT_CHARGINGMANAGEMENT_H_ */
