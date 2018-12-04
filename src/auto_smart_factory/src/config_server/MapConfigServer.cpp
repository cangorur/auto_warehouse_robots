/*
 * MapServer.cpp
 *
 *  Created on: 30.05.2017
 *      Author: jacob
 */

#include <boost/property_tree/json_parser.hpp>
#include <config_server/MapConfigServer.h>

#include <auto_smart_factory/Robot.h>

using namespace boost::property_tree;

unsigned int MapConfigServer::trayIdCounter = 0;

MapConfigServer::MapConfigServer() {
	ros::NodeHandle nh("~");
	std::string mapConfigFileName;

	if(!nh.getParam("map_config_file", mapConfigFileName)) {
		ROS_FATAL(
				"No map configuration file name given! Map server could not create map.");
		ros::shutdown();
		return;
	}

	//nh.setParam("occupancy_map_resolution", 0.5);  // setting the resolution for occupancy_map by Ansgar
	// the line above does not exist in current master, but works for adjusting the resolution

	// setup map configuration
	readMapConfig(mapConfigFileName);
	setupOccupancyMap();

	configService = nh.advertiseService("get_map_configuration",
	                                    &MapConfigServer::configCallback, this);
	mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1,
	                                                     true);
	mapPublisher.publish(warehouseConfig.occupancy_map);
}

MapConfigServer::~MapConfigServer() {
}

void MapConfigServer::readMapConfig(std::string file) {
	ptree configTree;

	try {
		read_json(file, configTree);
	} catch(json_parser::json_parser_error& e) {
		ROS_FATAL("Cannot read warehouse configuration file %s. Message: %s", file.c_str(), e.what());
		return;
	}

	// read general info
	warehouseConfig.width = configTree.get<float>("map.width");
	warehouseConfig.height = configTree.get<float>("map.height");

	// read tray config
	warehouseConfig.tray_geometry.width = configTree.get<float>(
			"tray_geometry.width");
	warehouseConfig.tray_geometry.height = configTree.get<float>(
			"tray_geometry.height");

	// read trays
	warehouseConfig.trays.clear();
	ptree trays = configTree.get_child("trays");
	for(const auto& t : trays) {
		auto_smart_factory::Tray tray;
		tray.id = getUniqueTrayId();
		tray.type = t.second.get<std::string>("type");
		tray.x = t.second.get<float>("x");
		tray.y = t.second.get<float>("y");
		tray.orientation = t.second.get<float>("orientation");
		tray.max_load = t.second.get<float>("max_load");
		tray.package_type = t.second.get<unsigned int>("package_type");

		warehouseConfig.trays.push_back(tray);
	}

	// read package pool information
	ptree pckPoolConfig = configTree.get_child("package_pool");
	const float packagePoolX = configTree.get<float>("package_pool.location.x");
	const float packagePoolY = configTree.get<float>("package_pool.location.y");
	const float packagePoolZ = configTree.get<float>("package_pool.location.z");
	auto_smart_factory::PackagePool pool;
	pool.drop_location.x = packagePoolX + pckPoolConfig.get<float>("relative_drop_location.x");
	pool.drop_location.y = packagePoolY + pckPoolConfig.get<float>("relative_drop_location.y");
	pool.drop_location.z = packagePoolZ + pckPoolConfig.get<float>("relative_drop_location.z");
	pool.stacking_area[0].x = packagePoolX + pckPoolConfig.get<float>("relative_stacking_area.x1");
	pool.stacking_area[0].y = packagePoolY + pckPoolConfig.get<float>("relative_stacking_area.y1");
	pool.stacking_area[0].z = packagePoolZ + pckPoolConfig.get<float>("relative_stacking_area.z");
	pool.stacking_area[1].x = packagePoolX + pckPoolConfig.get<float>("relative_stacking_area.x2");
	pool.stacking_area[1].y = packagePoolY + pckPoolConfig.get<float>("relative_stacking_area.y2");
	pool.stacking_area[1].z = pool.stacking_area[0].z;
	warehouseConfig.package_pool = pool;

	// read robots and idle positions
	warehouseConfig.robots.clear();
	warehouseConfig.idle_positions.clear();
	ptree robots = configTree.get_child("robots");
	for(const auto& r : robots) {
		auto_smart_factory::Robot robot;
		robot.id = r.second.get<std::string>("name");
		robot.type = r.second.get<std::string>("type");
		warehouseConfig.robots.push_back(robot);

		auto_smart_factory::IdlePosition idlePos;
		idlePos.id = r.second.get<std::string>("name");
		idlePos.pose.x = r.second.get<double>("idle_position.x");
		idlePos.pose.y = r.second.get<double>("idle_position.y");
		idlePos.pose.theta = r.second.get<double>("idle_position.orientation");
		warehouseConfig.idle_positions.push_back(idlePos);
	}
}

void MapConfigServer::addStaticObstacles() {
	float trayOffsetX = warehouseConfig.tray_geometry.width / 2.0;
	float trayOffsetY = warehouseConfig.tray_geometry.height / 2.0;

	for(const auto_smart_factory::Tray& tray : warehouseConfig.trays) {
		float x = tray.x - trayOffsetX;
		float y = tray.y - trayOffsetY;
		setRectangularObstacle(x, y, warehouseConfig.tray_geometry.width,
		                       warehouseConfig.tray_geometry.height,
		                       warehouseConfig.occupancy_map);
	}
}

void MapConfigServer::setupOccupancyMap() {
	// create map size with default resolution 0.1 m per cell
	ros::NodeHandle nh("~");
	float mapResolution = nh.param("occupancy_map_resolution", 0.1);      // this does not work as intended I guess

	float mapWidth = warehouseConfig.width;
	float mapHeight = warehouseConfig.height;

	// fill in map meta data
	warehouseConfig.occupancy_map.header.frame_id = "map";
	warehouseConfig.occupancy_map.header.stamp = ros::Time::now();
	warehouseConfig.occupancy_map.info.map_load_time = ros::Time::now();
	warehouseConfig.occupancy_map.info.resolution = mapResolution;
	warehouseConfig.occupancy_map.info.width = std::ceil(
			mapWidth / mapResolution);
	warehouseConfig.occupancy_map.info.height = std::ceil(
			mapHeight / mapResolution);
	warehouseConfig.occupancy_map.info.origin.position.x = 0;
	warehouseConfig.occupancy_map.info.origin.position.y = 0;
	warehouseConfig.occupancy_map.info.origin.position.z = 0;
	warehouseConfig.occupancy_map.info.origin.orientation.w = 1;
	warehouseConfig.occupancy_map.info.origin.orientation.x = 0;
	warehouseConfig.occupancy_map.info.origin.orientation.y = 0;
	warehouseConfig.occupancy_map.info.origin.orientation.z = 0;

	// initialize grid cells
	warehouseConfig.occupancy_map.data.clear();
	warehouseConfig.occupancy_map.data.resize(
			warehouseConfig.occupancy_map.info.width
			* warehouseConfig.occupancy_map.info.height, 0);

	// add occupied areas
	addStaticObstacles();
}

bool MapConfigServer::configCallback(
		auto_smart_factory::GetWarehouseConfig::Request& req,
		auto_smart_factory::GetWarehouseConfig::Response& res) {
	res.warehouse_configuration = warehouseConfig;
	return true;
}

void MapConfigServer::setRectangularObstacle(float x, float y, float width,
                                             float height, nav_msgs::OccupancyGrid& grid) {
	ROS_ASSERT(x >= 0);
	ROS_ASSERT(y >= 0);
	ROS_ASSERT(width >= 0);
	ROS_ASSERT(height >= 0);

	// discretize rectangle
	unsigned int discreteWidth = std::ceil(width / grid.info.resolution);
	unsigned int discreteHeight = std::ceil(height / grid.info.resolution);
	unsigned int discreteX = x / grid.info.resolution;
	unsigned int discreteY = y / grid.info.resolution;

	// set cell probabilities
	for(unsigned int w = discreteX; w < discreteX + discreteWidth; w++) {
		for(unsigned int h = discreteY; h < discreteY + discreteHeight; h++) {
			setMapCell(w, h, 100, grid);
		}
	}
}

void MapConfigServer::setMapCell(unsigned int x, unsigned int y,
                                 int8_t cellProbability, nav_msgs::OccupancyGrid& grid) {
	unsigned int index = y * grid.info.width + x;

	ROS_ASSERT(index < grid.data.size());

	grid.data.at(index) = cellProbability;
}

unsigned int MapConfigServer::getUniqueTrayId() {
	return trayIdCounter++;
}
