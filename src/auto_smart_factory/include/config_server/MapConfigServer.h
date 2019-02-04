/*
 * MapServer.h
 *
 *  Created on: 30.05.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_MAPSERVER_H_
#define AUTO_SMART_FACTORY_SRC_MAPSERVER_H_

#include "ros/ros.h"
#include "auto_smart_factory/WarehouseConfiguration.h"
#include "auto_smart_factory/GetWarehouseConfig.h"
#include <boost/property_tree/ptree.hpp>

/**
 * This class reads the warehouse configuration file and provides a service to deliver this
 * configuration data to other components.
 */
class MapConfigServer {
public:
	MapConfigServer();
	virtual ~MapConfigServer() = default;

protected:

	/** Map configuration retrieve service callback function
	 * @param req Request object
	 * @param res Response object
	 * @return Success of service call (always true) */
	bool configCallback(auto_smart_factory::GetWarehouseConfig::Request& req, auto_smart_factory::GetWarehouseConfig::Response& res);

	/** Reads the JSON formatted map/warehouse configuration from file.
	 * The configuration is stored internally and can by retrieved via service call.
	 * @param file The path to the JSON configuration file */
	void readMapConfig(std::string file);

	/* Adds a rectangular occupied obstacle to the occupancy map.
	 * This is used to add the trays and charging stations as static obstacles to the occupancy map.
	 *
	 * \note At the moment arbitrary rotation of the trays is not supported and tray edges must align with world coordinate axes.
	 * Rotating by multiples of 90 degrees is supported as long as the trays have square footprint.
	 *
	 * @param x X coordinate o the lower left point of the rectangle [meters]
	 * @param y Y coordinate o the lower left point of the rectangle [meters]
	 * @param width Width of the obstacle (in x direction) [meters]
	 * @param height Height of the obstacle (in y direction) [meters]
	 * @param grid The occupancy grid used to set the occupied cells
	 */
	void setRectangularObstacle(float x, float y, float width, float height, float rotation);

	/**  Adds the static obstacles defined in the map configuration to the occupancy map.
	 * This comprises trays and charging stations. */
	void addStaticObstacles();

	auto_smart_factory::WarehouseConfiguration warehouseConfig;
	ros::ServiceServer configService;
	
	// Create a new unique tray id.
	static unsigned int getUniqueTrayId();
	static unsigned int trayIdCounter;
};

#endif /* AUTO_SMART_FACTORY_SRC_MAPSERVER_H_ */
