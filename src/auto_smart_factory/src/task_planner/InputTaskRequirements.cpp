/*
 * InputTaskRequirements.cpp
 *
 *  Created on: 12.07.2017
 *      Author: jacob
 */

#include "task_planner/InputTaskRequirements.h"

using namespace auto_smart_factory;

InputTaskRequirements::InputTaskRequirements(PackageConfiguration pkgConfig,
                                             unsigned int inputTrayId)
		:
		TaskRequirements(pkgConfig), inputTrayId(inputTrayId) {
}

InputTaskRequirements::~InputTaskRequirements() {
}

bool InputTaskRequirements::isLegalSourceTray(const Tray& tray) const {
	bool legal = true;

	// is desired input tray
	legal &= (tray.id == inputTrayId);

	// is input tray
	legal &= (tray.type == "input");

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

	// is suitable
	//legal &= (pkgConfig.weight <= tray.max_load);

	return legal;
}

bool InputTaskRequirements::isLegalTargetTray(const Tray& tray) const {
	bool legal = true;

	// is storage tray
	legal &= (tray.type == "storage");

	// is suitable
	legal &= (pkgConfig.weight <= tray.max_load);

	if(!legal) {
		return false;
	}

	// test state dependent conditions
	TrayState trayState = getTrayState(tray.id);

	// is not occupied
	legal &= !trayState.occupied;

	// is not reserved
	legal &= trayState.available;

	return legal;
}

bool InputTaskRequirements::checkAllocatedSourceTray(const auto_smart_factory::Tray& tray) const {
	bool legal = true;

	// is desired input tray
	legal &= (tray.id == inputTrayId);

	// is input tray
	legal &= (tray.type == "input");

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

	// is suitable
	//legal &= (pkgConfig.weight <= tray.max_load);

	return legal;
}

bool InputTaskRequirements::checkAllocatedTargetTray(const auto_smart_factory::Tray& tray) const {
	bool legal = true;

	// is storage tray
	legal &= (tray.type == "storage");

	// is suitable
	legal &= (pkgConfig.weight <= tray.max_load);

	if(!legal) {
		return false;
	}

	// test state dependent conditions
	TrayState trayState = getTrayState(tray.id);

	// is not occupied
	legal &= !trayState.occupied;

	// is reserved (sic!)
	legal &= !trayState.available;

	return legal;
}

bool InputTaskRequirements::getRobotRequestType() const {
	return true;
}

unsigned int InputTaskRequirements::getKnownTrayId() const {
	return inputTrayId;
}
