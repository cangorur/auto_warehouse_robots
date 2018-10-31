/*
 * StorageManagement.cpp
 *
 *  Created on: 12.06.2017
 *      Author: jacob
 */

#include <storage_management/StorageManagement.h>
#include <auto_smart_factory/GetWarehouseConfig.h>
#include <auto_smart_factory/StorageUpdate.h>

StorageManagement::StorageManagement() {
	ros::NodeHandle pn("~");
	initServer = pn.advertiseService("init", &StorageManagement::init, this);
}

StorageManagement::~StorageManagement() {
}

bool StorageManagement::init(
		auto_smart_factory::InitStorageManagementRequest &req,
		auto_smart_factory::InitStorageManagementResponse &res) {
	if (!initialized) {
		initialized = initialize(req.warehouse_configuration,
				req.package_configurations);
		if (initialized)
			ROS_INFO("StorageManagement succesfully initialized!");
		else
			ROS_ERROR("StorageManagement initialization failed!");
	} else
		ROS_WARN("StorageManagement has already been initialized!");

	res.success = initialized;
	return true;
}

bool StorageManagement::initialize(
		auto_smart_factory::WarehouseConfiguration warehouse_configuration,
		std::vector<auto_smart_factory::PackageConfiguration> package_configurations) {
	ros::NodeHandle pn("~");

	// initialize data structure
	initializeStates(warehouse_configuration);

	// advertise get_storage_information service
	getStorageInfoServer = pn.advertiseService("get_storage_information",
			&StorageManagement::getStorageInformation, this);

	getTrayStateServer = pn.advertiseService("get_tray_state",
			&StorageManagement::getTrayState, this);

	// advertise reserve_storage service
	reserveTrayServer = pn.advertiseService("reserve_tray",
			&StorageManagement::reserveTray, this);

	// advertise end reservation service
	endTrayReservationServer = pn.advertiseService("end_reservation",
			&StorageManagement::endTrayReservation, this);

	setPackageServer = pn.advertiseService("set_package",
			&StorageManagement::setPackage, this);

	getPackageServer = pn.advertiseService("get_package",
			&StorageManagement::getPackage, this);

	// advertise storage info topic
	storageUpdatePublisher = pn.advertise<auto_smart_factory::StorageUpdate>(
			"storage_update", 10);

	// connect to tray sensors
	traySensorSubscriber = n.subscribe("/warehouse/tray_sensors", 1000,
			&StorageManagement::receiveTraySensorMsg, this);

	return true;
}

void StorageManagement::initializeStates(
		const auto_smart_factory::WarehouseConfiguration& config) {
	// initialize tray states
	for (const auto_smart_factory::Tray& tray : config.trays) {
		if (tray.type == "charging station") {
			// charging stations are not put in the storage state
			continue;
		}

		auto_smart_factory::TrayState t;
		t.id = tray.id;
		t.occupied = false;
		t.available = true;

		auto r = trayStates.insert(
				std::pair<TrayId, auto_smart_factory::TrayState>(t.id, t));
		if (!r.second) {
			ROS_FATAL(
					"[storage management] Input tray IDs must be unique. ID %d occurs at least twice.",
					tray.id);
		}
	}
}

bool StorageManagement::getStorageInformation(
		auto_smart_factory::GetStorageState::Request &req,
		auto_smart_factory::GetStorageState::Response &res) {
	// get current state
	res.state = packStorageState();
	return true;
}

bool StorageManagement::getTrayState(auto_smart_factory::GetTrayStateRequest &req,
		auto_smart_factory::GetTrayStateResponse &res) {
	try {
		res.state = trayStates.at(req.trayId);
		return true;
	} catch (std::out_of_range &e) {
		ROS_WARN("[storage management] Attempted to get state of inexistent tray (specified id: %d",
				req.trayId);
	}
	return false;
}

bool StorageManagement::reserveTray(
		auto_smart_factory::ReserveStorageTrayRequest &req,
		auto_smart_factory::ReserveStorageTrayResponse &res) {
	try {
		auto_smart_factory::TrayState &t = trayStates.at(req.id);

		if (t.available) {
			t.available = false;
			res.success = true;
			ROS_INFO(
					"[storage management] Reservation of tray %d (occupied: %s).",
					t.id, std::to_string(t.occupied).c_str());
			publishStorageUpdate(t,
					auto_smart_factory::StorageUpdate::RESERVATION);
		} else {
			// tray is not available
			ROS_INFO("[storage management] Reserving storage tray failed: Tray is not available.");
			res.success = false;
		}
	} catch (std::out_of_range &e) {
		// tray does not exist
		ROS_ERROR("[storage management] Attempted to reserve inexistent tray (specified id: %d)",
				req.id);
		res.success = false;
	}

	return true;
}

bool StorageManagement::endTrayReservation(
		auto_smart_factory::ReserveStorageTrayRequest &req,
		auto_smart_factory::ReserveStorageTrayResponse &res) {
	try {
		auto_smart_factory::TrayState &t = trayStates.at(req.id);

		if (!t.available) {
			// un-reserve tray
			t.available = true;
			res.success = true;
			ROS_INFO(
					"[storage management] De-reservation of tray %d (occupied: %s).",
					t.id, std::to_string(t.occupied).c_str());
			publishStorageUpdate(t,
					auto_smart_factory::StorageUpdate::DERESERVATION);
		} else {
			// tray is not reserved
			ROS_WARN(
					"[storage management] Ending reservation of tray failed: Tray is not reserved.");
			res.success = false;
		}

	} catch (std::out_of_range &e) {
		// tray does not exist
		ROS_ERROR(
				"[storage management] Attempted to end reservation of inexistent tray (specified id: %d)",
				req.id);
		res.success = false;
	}

	return true;
}

bool StorageManagement::setPackage(auto_smart_factory::SetPackageRequest &req,
		auto_smart_factory::SetPackageResponse &res) {
	try {
		auto_smart_factory::TrayState &t = trayStates.at(req.trayId);
		t.package = req.pkg;
		ROS_INFO(
				"[storage management] Set package (id: %d, type: %d) at tray %d",
				t.package.id, t.package.type_id, t.id);
		return true;
	} catch (std::out_of_range &e) {
		// tray does not exist
		ROS_ERROR(
				"[storage management] Attempted to set package at inexistent tray (specified id: %d)",
				req.trayId);
		return false;
	}
}

bool StorageManagement::getPackage(auto_smart_factory::GetPackageRequest &req,
		auto_smart_factory::GetPackageResponse &res) {
	try {
		auto_smart_factory::TrayState &t = trayStates.at(req.trayId);
		res.pkg = t.package;
		return true;
	} catch (std::out_of_range &e) {
		// tray does not exist
		ROS_ERROR(
				"[storage management] Attempted to set package at inexistent tray (specified id: %d)",
				req.trayId);
		return false;
	}
}

void StorageManagement::receiveTraySensorMsg(
		const auto_smart_factory::TraySensor &msg) {
	try {
		auto_smart_factory::TrayState &state = trayStates.at(msg.tray_id);

		if (msg.occupied) {
			if (state.occupied) {
				ROS_ERROR(
						"[storage management] Package was put into an already occupied tray (id: %d)",
						msg.tray_id);
			}

			if (state.available) {
				ROS_ERROR(
						"[storage management] Tray was not reserved before a package was put into it (id: %d)",
						msg.tray_id);
			}

			auto_smart_factory::Package pkg = strToPkg(msg.package);
			if (pkg.id != state.package.id
					|| pkg.type_id != state.package.type_id) {
				ROS_ERROR(
						"[storage management] Package put into tray %d does not match the package that was announced! (announced: id=%d type=%d, inserted: id=%d type=%d)",
						state.id, state.package.id, state.package.type_id,
						pkg.id, pkg.type_id);
				return;
			}

			state.occupied = true;

			ROS_INFO("[storage management] Package was put into tray %d",
					msg.tray_id);

			publishStorageUpdate(state,
					auto_smart_factory::StorageUpdate::OCCUPATION);
		} else {
			if (!state.occupied) {
				ROS_ERROR(
						"[storage management] Package was removed from a tray that was not marked as occupied (id: %d)",
						msg.tray_id);
			}

			if (state.available) {
				ROS_ERROR(
						"[storage management] Tray was not reserved before a package was removed from it (id: %d)",
						msg.tray_id);
			}
			
			state.occupied = false;

			ROS_INFO("[storage management] Package was removed from tray %d",
					msg.tray_id);

			publishStorageUpdate(state,
					auto_smart_factory::StorageUpdate::DEOCCUPATION);
		}
	} catch (std::out_of_range &e) {
		ROS_ERROR("[storage management] Tray sensor message with invalid tray id (id: %d)",
				msg.tray_id);
	}
}

auto_smart_factory::StorageState StorageManagement::packStorageState() const {
	auto_smart_factory::StorageState msg;

	// set current time
	msg.stamp = ros::Time::now();

	// insert states of trays
	for (const auto& trayState : trayStates) {
		msg.tray_states.push_back(trayState.second);
	}

	return msg;
}

void StorageManagement::publishStorageUpdate(
		auto_smart_factory::TrayState &tray_state, uint8_t action) {
	auto_smart_factory::StorageUpdate update;
	update.stamp = ros::Time::now();
	update.state = tray_state;
	update.action = action;
	storageUpdatePublisher.publish(update);
}

std::vector<std::string> split(std::string &text, char sep) {
	std::vector<std::string> tokens;
	std::size_t start = 0, end = 0;
	while ((end = text.find(sep, start)) != std::string::npos) {
		tokens.push_back(text.substr(start, end - start));
		start = end + 1;
	}
	tokens.push_back(text.substr(start));
	return tokens;
}

auto_smart_factory::Package StorageManagement::strToPkg(std::string s) {
	auto_smart_factory::Package package;
	std::vector<std::string> ids = split(split(s, 'g')[1], '_');
	package.type_id = std::stoi(ids[0]);
	package.id = std::stoi(ids[1]);
	return package;
}
