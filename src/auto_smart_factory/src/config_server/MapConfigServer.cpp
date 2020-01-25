/*
 * MapServer.cpp
 *
 *  Created on: 30.05.2017
 *      Author: jacob
 */

#include <boost/property_tree/json_parser.hpp>
#include "Math.h"
#include "config_server/MapConfigServer.h"

using namespace boost::property_tree;

unsigned int MapConfigServer::trayIdCounter = 0;

MapConfigServer::MapConfigServer() {
	ros::NodeHandle nh("~");
	std::string mapConfigFileName;

	if(!nh.getParam("map_config_file", mapConfigFileName)) {
		ROS_FATAL("No map configuration file name given! Map server could not create map.");
		ros::shutdown();
		return;
	}

	// setup map configuration
	readMapConfig(mapConfigFileName);
	addStaticObstacles();

	configService = nh.advertiseService("get_map_configuration", &MapConfigServer::configCallback, this);
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
	
	warehouseConfig.map_configuration.width = configTree.get<float>("map.width");
	warehouseConfig.map_configuration.height = configTree.get<float>("map.height");
	warehouseConfig.map_configuration.margin = configTree.get<float>("map.margin");
	warehouseConfig.map_configuration.resolutionThetaStar = configTree.get<float>("map.theta_star_resolution");	
	
	// read tray config
	warehouseConfig.tray_geometry.width = configTree.get<float>("tray_geometry.width");
	warehouseConfig.tray_geometry.height = configTree.get<float>("tray_geometry.height");
	
	mergeObstacles = configTree.get<bool>("map.mergeObstacles");
	ROS_INFO("Merging obstacles");

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
	std::vector<Rectangle> rectangles;

	for(const auto_smart_factory::Tray& tray : warehouseConfig.trays) {
		setRectangularObstacle(rectangles, tray.x, tray.y, warehouseConfig.tray_geometry.width, warehouseConfig.tray_geometry.height, 0);
	}

	setRectangularObstacle(rectangles, 6, 11.25, 3.5, 0.25, 0);
	setRectangularObstacle(rectangles, 10.75, 11.25, 3.5, 0.25, 0);

	setRectangularObstacle(rectangles, 6, 9.25, 3.5, 0.25, 0);
	setRectangularObstacle(rectangles, 10.75, 9.25, 3.5, 0.25, 0);

	setRectangularObstacle(rectangles, 6, 7.75, 3.5, 0.25, 0);
	setRectangularObstacle(rectangles, 10.75, 7.75, 3.5, 0.25, 0);

	setRectangularObstacle(rectangles, 6, 5.75, 3.5, 0.25, 0);
	setRectangularObstacle(rectangles, 10.75, 5.75, 3.5, 0.25, 0);

	setRectangularObstacle(rectangles, 6, 4.25, 3.5, 0.25, 0);
	setRectangularObstacle(rectangles, 10.75, 4.25, 3.5, 0.25, 0);

	setRectangularObstacle(rectangles, 6, 2.25, 3.5, 0.25, 0);
	setRectangularObstacle(rectangles, 10.75, 2.25, 3.5, 0.25, 0);


	// Left and right map margin
	setRectangularObstacle(rectangles,1.75, warehouseConfig.map_configuration.height/2, 2,warehouseConfig.map_configuration.height, 0);
	setRectangularObstacle(rectangles,14.75, (warehouseConfig.map_configuration.height/2), 2,warehouseConfig.map_configuration.height, 0);
	
	// Top and bottom map margin
	setRectangularObstacle(rectangles, warehouseConfig.map_configuration.width/2, 13.5,  warehouseConfig.map_configuration.width, 2.0, 0);
	setRectangularObstacle(rectangles, warehouseConfig.map_configuration.width/2, 0.0, warehouseConfig.map_configuration.width, 2.0, 0);
	

	// Convert
	warehouseConfig.map_configuration.obstacles.clear();
	
	for(const Rectangle& r : rectangles) {
		auto_smart_factory::Rectangle rectangle;
		rectangle.posX = r.getPosition().x;
		rectangle.posY = r.getPosition().y;
		rectangle.sizeX = r.getSize().x;
		rectangle.sizeY = r.getSize().y;
		rectangle.rotation = r.getRotation();
		rectangle.ownerId = -1;

		warehouseConfig.map_configuration.obstacles.push_back(rectangle);
	}	
}

void MapConfigServer::setRectangularObstacle(std::vector<Rectangle>& rectangles, float x, float y, float width, float height, float rotation) {
	Rectangle r1 = Rectangle(Point(x, y), Point(width, height), rotation);
	
	if(mergeObstacles) {
		// Try to merge with existing ones
		auto iter = rectangles.begin();
		while(iter != rectangles.end()) {
			if(!(r1.getMinXInflated() > iter->getMaxXInflated() || r1.getMinYInflated() > iter->getMaxYInflated() ||
			     iter->getMinXInflated() > r1.getMaxXInflated() || iter->getMinYInflated() > r1.getMaxYInflated())) {
				// Construct new rectangle
				double minX = std::min(r1.getMinXInflated(), iter->getMinXInflated()) + ROBOT_RADIUS;
				double maxX = std::max(r1.getMaxXInflated(), iter->getMaxXInflated()) - ROBOT_RADIUS;
				double minY = std::min(r1.getMinYInflated(), iter->getMinYInflated()) + ROBOT_RADIUS;
				double maxY = std::max(r1.getMaxYInflated(), iter->getMaxYInflated()) - ROBOT_RADIUS;

				r1 = Rectangle(Point((maxX + minX) * 0.5f, (maxY + minY) * 0.5f), Point(maxX - minX, maxY - minY), 0);
				rectangles.erase(iter);
				break;
			}
			iter++;
		}
	}
	
	rectangles.push_back(r1);
}

bool MapConfigServer::configCallback(auto_smart_factory::GetWarehouseConfig::Request& req, auto_smart_factory::GetWarehouseConfig::Response& res) {
	res.warehouse_configuration = warehouseConfig;
	return true;
}

unsigned int MapConfigServer::getUniqueTrayId() {
	return trayIdCounter++;
}
