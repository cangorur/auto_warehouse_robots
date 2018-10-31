/*
 * TrayAllocator.cpp
 *
 *  Created on: 11.07.2017
 *      Author: jacob
 */

#include <storage_management/TrayAllocator.h>

#include <ros/ros.h>
#include <auto_smart_factory/ReserveStorageTray.h>
#include <auto_smart_factory/GetPackage.h>
#include <auto_smart_factory/SetPackage.h>

using namespace auto_smart_factory;

TrayAllocator::TrayAllocator(unsigned int trayId) : trayId(trayId) {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ReserveStorageTray>(
			"/storage_management/reserve_tray");
	ReserveStorageTray srv;
	srv.request.id = trayId;

	valid = (client.call(srv) && srv.response.success);
}

TrayAllocator::~TrayAllocator() {
	if (valid) {
		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<ReserveStorageTray>(
				"/storage_management/end_reservation");
		ReserveStorageTray srv;
		srv.request.id = trayId;

		// end reservation
		client.call(srv);

		valid = false;
	}
}

bool TrayAllocator::isValid() const {
	return valid;
}

unsigned int TrayAllocator::getId() const {
	return trayId;
}

bool TrayAllocator::setPackage(const auto_smart_factory::Package &pkg) {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<SetPackage>(
			"/storage_management/set_package");
	SetPackage srv;
	srv.request.trayId = trayId;
	srv.request.pkg = pkg;

	// set package
	return client.call(srv);
}

auto_smart_factory::Package TrayAllocator::getPackage() {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<GetPackage>(
			"/storage_management/get_package");
	GetPackage srv;
	srv.request.trayId = trayId;

	// set package
	client.call(srv);

	return srv.response.pkg;
}

TrayAllocatorPtr TrayAllocator::allocateTray(unsigned int trayId) {
	return std::make_shared<TrayAllocator>(trayId);
}
