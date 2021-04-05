/*
 * OutputTaskRequirements.h
 *
 *  Created on: 12.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_OUTPUTTASKREQUIREMENTS_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_OUTPUTTASKREQUIREMENTS_H_

#include "task_planner/TaskRequirements.h"

/**
 * This class encapsulates the requirements to resources that an output request has.
 */
class OutputTaskRequirements : public TaskRequirements {
public:
	/**
	 * Creates output request requirements.
	 * @param pkgConfig The package configuration of the requested package
	 * @param outputTrayId The id of the output tray
	 */
	OutputTaskRequirements(auto_smart_factory::PackageConfiguration pkgConfig, unsigned int outputTrayId);

	virtual ~OutputTaskRequirements();

	/**
	 * Checks if the specified tray is the legal source tray, i.e.
	 * it is an input or storage tray containing a package with correct type
	 * and it is not reserved.
	 * @param tray The tray to be checked
	 * @return Result of the check
	 */
	bool isLegalSourceTray(const auto_smart_factory::Tray& tray) const;

	/**
	 * Checks if the specified tray is the legal target tray, i.e.
	 * it is the fixed output tray with the outputTrayId,
	 * is not occupied and it is not reserved.
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
	 * @return Always false
	 */
	bool getRobotRequestType() const;

	/**
	 * Returns the id of the fixed output tray.
	 * @return Tray id
	 */
	unsigned int getKnownTrayId() const;

private:
	/// The associated output tray
	unsigned int outputTrayId;
};

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_OUTPUTTASKREQUIREMENTS_H_ */
