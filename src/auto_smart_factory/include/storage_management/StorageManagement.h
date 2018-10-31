/*
 * StorageManagement.h
 *
 *  Created on: 12.06.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_STORAGEMANAGEMENT_H_
#define AUTO_SMART_FACTORY_SRC_STORAGEMANAGEMENT_H_

#include <vector>
#include <ros/ros.h>

#include <auto_smart_factory/WarehouseConfiguration.h>
#include <auto_smart_factory/GetStorageState.h>
#include <auto_smart_factory/GetTrayState.h>
#include <auto_smart_factory/ReserveStorageTray.h>
#include <auto_smart_factory/SetPackage.h>
#include <auto_smart_factory/GetPackage.h>
#include <auto_smart_factory/NewPackageOutput.h>
#include <auto_smart_factory/InitStorageManagement.h>
#include <std_msgs/String.h>
#include <auto_smart_factory/TraySensor.h>

/// Tray id type
typedef uint32_t TrayId;

/**
 * This component manages the states of the warehouse trays.
 * It also provides services to retrieve and manipulate this state
 * and reserve trays.
 */
class StorageManagement {
public:
	StorageManagement();
	virtual ~StorageManagement();

protected:

	/**
	 * Handles the initialization request from the warehouse management.
	 * @param req Request object with configurations
	 * @param res Response object
	 * @return Always true
	 */
	bool init(auto_smart_factory::InitStorageManagementRequest &req,
			auto_smart_factory::InitStorageManagementResponse &res);

	/**
	 * Initializes the internal storage state using the warehouse configuration.
	 * @param warehouse_configuration The warehouse configuration
	 * @param package_configurations The package configurations
	 * @return Success (always true)
	 */
	bool initialize(
			auto_smart_factory::WarehouseConfiguration warehouse_configuration,
			std::vector<auto_smart_factory::PackageConfiguration> package_configurations);

	/**
	 * Initializes the states.
	 *
	 * \note All trays are free initially.
	 *
	 * @param config The warehouse configuration
	 */
	void initializeStates(
			const auto_smart_factory::WarehouseConfiguration& config);

	/**
	 * Service handler to return the storage state.
	 * @param req Request object
	 * @param res Response object containing state
	 * @return Success (always true)
	 */
	bool getStorageInformation(auto_smart_factory::GetStorageState::Request &req,
			auto_smart_factory::GetStorageState::Response &res);

	/**
	 * Service handler to return the state of a single tray.
	 * @param req Request object with specified tray
	 * @param res Response object with tray state
	 * @return Success (true if specified tray exists)
	 */
	bool getTrayState(auto_smart_factory::GetTrayStateRequest &req,
			auto_smart_factory::GetTrayStateResponse &res);

	/**
	 * Service handler to reserve a specified tray.
	 * @param req Request object with specified tray
	 * @param res Response object with success (true if tray exists and is not already reserved)
	 * @return Always true
	 */
	bool reserveTray(auto_smart_factory::ReserveStorageTrayRequest &req,
			auto_smart_factory::ReserveStorageTrayResponse &res);

	/**
	 * Service handler to end reservation of tray.
	 * @param req Request object with specified tray
	 * @param res Response object with success (true if tray exists and was reserved before)
	 * @return Always true
	 */
	bool endTrayReservation(auto_smart_factory::ReserveStorageTrayRequest &req,
			auto_smart_factory::ReserveStorageTrayResponse &res);

	/**
	 * Service handler to set package information for a specific tray.
	 * @param req Request object with package information and specified tray
	 * @param res Response object
	 * @return Success (true if tray exists)
	 */
	bool setPackage(auto_smart_factory::SetPackageRequest &req,
			auto_smart_factory::SetPackageResponse &res);

	/**
	 * Service handler to get package information for a specific tray.
	 * @param req Request object with specified tray
	 * @param res Response object with package information
	 * @return Success (true if tray exists)
	 */
	bool getPackage(auto_smart_factory::GetPackageRequest &req, auto_smart_factory::GetPackageResponse &res);

	/**
	 * Message receive handler for tray sensor messages. These messages are sent
	 * when the occupation state of a tray changes.
	 * The internal tray states are updated accordingly.
	 * @param msg The sensor message
	 */
	void receiveTraySensorMsg(const auto_smart_factory::TraySensor &msg);

	/**
	 * Packs current storage state into the message format.
	 * @return Storage state message
	 */
	auto_smart_factory::StorageState packStorageState() const;

	/**
	 * Publishes the update of a tray state.
	 * This can be used by other components to trigger actions.
	 * @param tray_state New state of the tray
	 * @param action The action that led to this update (reservation, de-reservation, occupation or de-occupation)
	 */
	void publishStorageUpdate(auto_smart_factory::TrayState &tray_state,
			uint8_t action);

	/**
	 * Converts a string of the format 'pkgx_x' to package information.
	 * @param s The string
	 * @return The package information (id, type)
	 */
	static auto_smart_factory::Package strToPkg(std::string s);

protected:
	ros::NodeHandle n;

	bool initialized = false;
	ros::ServiceServer initServer;
	ros::ServiceServer getStorageInfoServer;
	ros::ServiceServer getTrayStateServer;
	ros::ServiceServer reserveTrayServer;
	ros::ServiceServer endTrayReservationServer;
	ros::ServiceServer setPackageServer;
	ros::ServiceServer getPackageServer;

	ros::Subscriber traySensorSubscriber;
	ros::Publisher storageUpdatePublisher;

	/// Internal tray state
	std::map<TrayId, auto_smart_factory::TrayState> trayStates;
};

#endif /* AUTO_SMART_FACTORY_SRC_STORAGEMANAGEMENT_H_ */
