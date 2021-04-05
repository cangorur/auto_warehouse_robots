/*
 * InputTaskRequirements.h
 *
 *  Created on: 12.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_INPUTTASKREQUIREMENTS_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_INPUTTASKREQUIREMENTS_H_

#include "task_planner/TaskRequirements.h"

/**
 * This class encapsulates the requirements to resources that an input request has.
 */
class InputTaskRequirements : public TaskRequirements {
public:
	/**
	 * Creates input request requirements.
	 * @param pkgConfig The package configuration of the input package
	 * @param inputTrayId The id of the input tray
	 */
	InputTaskRequirements(auto_smart_factory::PackageConfiguration pkgConfig, unsigned int inputTrayId);

	virtual ~InputTaskRequirements();

	/**
	 * Checks if the specified tray is the legal source tray, i.e.
	 * it is the fixed input tray with the inputTrayId,
	 * it is occupied by the right package type and it is not reserved.
	 * @param tray The tray to be checked
	 * @return Result of the check
	 */
	bool isLegalSourceTray(const auto_smart_factory::Tray& tray) const;

	/**
	 * Checks if the specified tray is the legal target tray, i.e.
	 * it is a free storage tray with an appropriate maximum load,
	 * and it is not reserved.
	 * @param tray The tray to be checked
	 * @return Result of the check
	 */
	bool isLegalTargetTray(const auto_smart_factory::Tray& tray) const;

	/**
	 * Checks if an allocated (reserved) tray still fulfills the requirements for the source tray.
	 * This is the same as isLegalSourceTray except that the tray now should be reserved.
	 * @param tray The allocated tray to be checked
	 * @return Result of the check
	 */
	bool checkAllocatedSourceTray(const auto_smart_factory::Tray& tray) const;

	/**
	 * Checks if an allocated (reserved) tray still fulfills the requirements for the target tray.
	 * This is the same as isLegalTargetTray except that the tray now should be reserved.
	 * @param tray The allocated tray to be checked
	 * @return Result of the check
	 */
	bool checkAllocatedTargetTray(const auto_smart_factory::Tray& tray) const;

	/**
	 * This is only used to determine which request to send to the agent.
	 *
	 * \todo It would be nicer to make also the request to the agent generic,
	 * i.e. independent of the type of input/output request.
	 *
	 * @return Always true
	 */
	bool getRobotRequestType() const;

	/**
	 * Returns the id of the fixed input tray.
	 * @return Tray id
	 */
	unsigned int getKnownTrayId() const;

private:
	/// The associated input tray
	unsigned int inputTrayId;
};

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_INPUTTASKREQUIREMENTS_H_ */
