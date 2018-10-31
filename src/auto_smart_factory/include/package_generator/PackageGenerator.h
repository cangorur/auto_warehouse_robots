#ifndef AUTO_SMART_FACTORY_SRC_PACKAGEGENERATOR_H_
#define AUTO_SMART_FACTORY_SRC_PACKAGEGENERATOR_H_

#include "ros/ros.h"
#include <string>
#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <deque>
#include "auto_smart_factory/InitPackageGenerator.h"
#include <auto_smart_factory/GetStorageState.h>
#include <auto_smart_factory/GetTrayState.h>
#include <auto_smart_factory/NewPackageGenerator.h>
#include <auto_smart_factory/NewPackageInput.h>
#include <auto_smart_factory/NewPackageOutput.h>
#include <auto_smart_factory/MovePackage.h>
#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/PackageConfiguration.h>
#include <auto_smart_factory/StorageUpdate.h>
#include <auto_smart_factory/Package.h>
#include <storage_management/TrayAllocator.h>

/**
 * The package generator component simulates package sources that lead to incoming packages at input trays
 * as well as package demands at the output trays, such that a continous package flow can be achieved.
 * Removes packages from output trays automatically.
 * This component could be replaced by or integrated into a real package flow.
 */
class PackageGenerator{
public:
	/**
	 * Default constructor.
	 * Sets up the initialize service.
	 */
	PackageGenerator();
	virtual ~PackageGenerator();

	/**
	 * Called every tick to keep up the generation of requests to continue the package flow.
	 * Checks whether the storage state is needed.
	 * Checks whether it's time for next request generation.
	 */
	void update();

protected:
	/**
	 * Initialize service handler.
	 * @param req Request object
	 * @param res Response object
	 * @return True if initialization was successful
	 */
	bool init(auto_smart_factory::InitPackageGenerator::Request  &req, 
			auto_smart_factory::InitPackageGenerator::Response &res);
	/**
	 * Initializes package generator, intern package list, sets up request plan
	 * & subscribes to storage update topic.
	 * @param warehouse_configuration information about the current map
	 * @param package_configurations all different package configurations
	 * @return True if initialization was successful
	 */
	bool initialize(auto_smart_factory::WarehouseConfiguration warehouse_configuration, 
			std::vector<auto_smart_factory::PackageConfiguration> package_configurations);
	/**
	 * Initializes intern package list: externalPackages.
	 * For every tray a package of each type is considered to exist.
	 */
	void initializePackages();

	/**
	 * Predefines request plan for more comparability to get meaningful results.
	 */
	void initializeRequestPlan();

	/**
	 * Requests the current storage state at the storage management.
	 * Called if there is no storage state yet.
	 */
	bool getStorageInformation();

	/**
	 * Receive storage updates & updates the internal storage state.
	 * @param msg Storage update
	 */
	void updateTrayState(const auto_smart_factory::StorageUpdate &msg);

	/**
	 * Tells whether the time has come for a new request generation.
	 * Uses the breakDuration for that decision.
	 * @return True if the next request should be generated.
	 */
	bool isTimeForGeneration();

	/**
	 * Resets the timer after another request generation took place.
	 */
	void updateTimer();

	/**
	 * A service to randomly generate new requests, depending on the storage state & package types the input &
	 * output trays are able to carry.
	 * @todo This service has not been tested yet! See the factory_gateway for its example (yet not implemented) usage
	 */
	bool generateService(auto_smart_factory::NewPackageGenerator::Request &req, 
				auto_smart_factory::NewPackageGenerator::Response &res);
	
	/**
	 * The function that randomly generates new requests (see the service version above), 
	 * depending on the storage state & package types the input & output trays are able to carry.
	 * This function is called under the Update function continuously to generate a new package.  
	 * @todo here will be updated to get the trays not only through their inputs but also their types ! (line 100)
	 * @todo Refer to line 323 to reserve tray-8 (the input-tray#1) for the 3D printer and conveyor belt scenario! 
	 * See newPackageInputOnConveyor service
	 */
	void generate();

	/**
	 * Returns all available & unoccupied trays of the given tray type.
	 * @param tray_type: the type of the tray. Valid values: "all", "input", "output" and "storage"
 	 * @return list of tray states of the given tray type
	 */
	std::vector<auto_smart_factory::TrayState> getFreeStorages(std::string tray_type);

	/**
	 * Tries to create a new package input request processing tray allocation, package movement &
	 * taskplanner request.
	 * @param tray: input tray that is considered for new input
	 * @param package: concrete package to move to the given input tray
	 * @return True if new request has been generated
	 */
	bool newPackageInput(auto_smart_factory::Tray tray, auto_smart_factory::Package package); 
	
	/**
	 * Tries to create a new package input ON THE CONVEYOR BELT request processing tray allocation, package movement &
	 * taskplanner request.
	 * @todo currently the locations are hard coded to move the pkg on conveyor which runs and makes it fall to input tray-01 
	 * !! this will be fixed according to the trays !
	 * @param tray: input tray that is considered for new input
	 * @param package: concrete package to move to the given input tray
	 * @return True if new request has been generated
	 */
	bool newPackageInputOnConveyor(auto_smart_factory::Tray tray, auto_smart_factory::Package package);

	/**
	 * Tries to create a new package output request processing tray allocation & taskplanner request.
	 * @param output_tray_id: output tray that is considered for new output 
	 * @param package: package with desired package type
	 * @return True if new request has been generated
	 */
	bool newPackageOutput(int output_tray_id, auto_smart_factory::Package package); 

	/**
	 * Clears output tray after it has been occupied.
	 * Package is moved to a fix position below the map (z = -1). This is possible as long as 
	 * physics for packages are turned off. Moved package is put back to externalPackages list.
	 * @param tray_state: state of the tray that could be cleared
	 */
	void clearOutput(auto_smart_factory::TrayState tray_state);

	/**
	 * Requests package manipulator component to move the given package to the given position 
	 * on the current map.
	 * @param x: x coordinate of the target location
	 * @param y: y coordinate of the target location
	 * @param z: z coordinate of the target location
	 * @param package: the specified package to move
	 * return True if package has been moved succesfully to the given position
	 */
	bool movePackage(float x, float y, float z, auto_smart_factory::Package package);

	/**
	 * Puts back the package just removed from an output tray to the externalPackages list.
	 * @param package: specified package to put back
	 */
	void putBackPackage(auto_smart_factory::Package package);

	/**
	 * Returns the tray with the specified id.
	 * @param id: tray id
	 * @return tray with the given id
	 */
	auto_smart_factory::Tray getTray(unsigned int id);

	/**
	 * Requests tray state of the tray with the given id.
	 * @param tray_id: desired tray id
	 * @return tray state of the tray with the given id
	 */
	auto_smart_factory::TrayState getTrayState(unsigned int tray_id);
	
	/// ROS Nodehandle
	ros::NodeHandle n;

	/// Server for initialization
	ros::ServiceServer initSrv; 
	
	/// Package generator service
	ros::ServiceServer generateNewPackageServer;

	/// Subscriber to the storage update topic
	ros::Subscriber storageUpdateSub;

	/// Client for the tray state service request
	ros::ServiceClient trayStateClient;

	/// Flag that shows if the package generator already has been initialized
	bool initialized = false;

	/// Flag that shows if the package generator already received the current storage state
	bool hasStorageState = false;

	/// Flag that shows if a request generation is just happening
	bool generating = false;

	/// Information about the current map 
	auto_smart_factory::WarehouseConfiguration warehouseConfig;

	/// All package configurations
	std::vector<auto_smart_factory::PackageConfiguration> packageConfigs;

	/// Storage state of the current warehouse
	auto_smart_factory::StorageState storageState;

	/// Lists of package ids of each type that are currently not inside the warehouse
	std::vector<std::deque<int>> externalPackages;

	/// Timestamp of last package generation
	unsigned long lastTimestamp = 0;

	/// Time frequeny for generating packages in seconds
	unsigned long breakDuration = 5;

	/// Flag if the request plans should be considered
	bool doRequestPlans = false;

	/// Predefined request plan (request type, package type)
	typedef std::pair<std::string, unsigned int> RequestPlan;

	/// List of request plans
	std::deque<RequestPlan> requestPlans;
};

#endif /* AUTO_SMART_FACTORY_SRC_PACKAGEGENERATOR_H_ */
