/*
 * OutputTaskRequirements.cpp
 *
 *  Created on: 12.07.2017
 *      Author: jacob
 */

#include "task_planner/OutputTaskRequirements.h"

using namespace auto_smart_factory;

OutputTaskRequirements::OutputTaskRequirements(
		auto_smart_factory::PackageConfiguration pkgConfig,
		unsigned int outputTrayId)
		:
		TaskRequirements(pkgConfig), outputTrayId(outputTrayId) {
}

OutputTaskRequirements::~OutputTaskRequirements() {
}

bool OutputTaskRequirements::isLegalSourceTray(const Tray& tray) const {
	bool legal = true;

	// is storage or input tray
	legal &= (tray.type == "storage" || tray.type == "input");

	// is suitable
	//legal &= (pkgConfig.weight <= tray.max_load);

	if(!legal) {
		return false;
	}

	// test state dependent conditions
	TrayState trayState = getTrayState(tray.id);

	// is occupied
	legal &= trayState.occupied;

	// is not reserved
	legal &= trayState.available;

	// contains correct package type
	legal &= (trayState.package.type_id == pkgConfig.id);

	return legal;
}

bool OutputTaskRequirements::isLegalTargetTray(const Tray& tray) const {
	bool legal = true;

	// is desired output tray
	legal &= (tray.id == outputTrayId);

	// is output tray
	legal &= (tray.type == "output");

	if(!legal) {
		return false;
	}

	// test state dependent conditions
	TrayState trayState = getTrayState(tray.id);

	// is not occupied
	legal &= !trayState.occupied;

	// is not reserved
	legal &= trayState.available;

	// is suitable
	//legal &= (pkgConfig.weight <= tray.max_load);

	return legal;
}

bool OutputTaskRequirements::checkAllocatedSourceTray(
		const auto_smart_factory::Tray& tray) const {
	bool legal = true;

	// is storage or input tray
	legal &= (tray.type == "storage" || tray.type == "input");

	// is suitable
	//legal &= (pkgConfig.weight <= tray.max_load);

	if(!legal) {
		return false;
	}

	// test state dependent conditions
	TrayState trayState = getTrayState(tray.id);

	// is occupied
	legal &= trayState.occupied;

	// is reserved (sic!)
	legal &= !trayState.available;

	// contains correct package type
	legal &= (trayState.package.type_id == pkgConfig.id);

	return legal;
}

bool OutputTaskRequirements::checkAllocatedTargetTray(
		const auto_smart_factory::Tray& tray) const {
	bool legal = true;

	// is desired output tray
	legal &= (tray.id == outputTrayId);

	// is output tray
	legal &= (tray.type == "output");

	if(!legal) {
		return false;
	}

	// test state dependent conditions
	TrayState trayState = getTrayState(tray.id);

	// is not occupied
	legal &= !trayState.occupied;

	// is reserved (sic!)
	legal &= !trayState.available;

	// is suitable
	//legal &= (pkgConfig.weight <= tray.max_load);

	return legal;
}

bool OutputTaskRequirements::getRobotRequestType() const {
	return false;
}

unsigned int OutputTaskRequirements::getKnownTrayId() const {
	return outputTrayId;
}
