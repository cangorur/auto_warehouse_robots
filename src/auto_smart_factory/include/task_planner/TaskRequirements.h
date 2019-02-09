/*
 * TaskRequirements.h
 *
 *  Created on: 11.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKREQUIREMENTS_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKREQUIREMENTS_H_

#include "auto_smart_factory/PackageConfiguration.h"
#include "auto_smart_factory/RobotConfiguration.h"
#include "auto_smart_factory/Tray.h"
#include "auto_smart_factory/TrayState.h"

/**
 * This class encapsulates the requirements to resources that all types of requests have.
 */
class TaskRequirements {
public:
	/**
	 * Creates task requirements.
	 * @param pkgConfig The package configuration of either the input package or the requested output package
	 */
	TaskRequirements(auto_smart_factory::PackageConfiguration pkgConfig);

	virtual ~TaskRequirements();

	/**
	 * Checks if the specified tray is the legal source tray.
	 * @param tray The tray to be checked
	 * @return Result of the check
	 */
	virtual bool isLegalSourceTray(const auto_smart_factory::Tray& tray) const = 0;

	/**
	 * Checks if the specified tray is the legal target tray.
	 * @param tray The tray to be checked
	 * @return Result of the check
	 */
	virtual bool isLegalTargetTray(const auto_smart_factory::Tray& tray) const = 0;

	/**
	 * Checks if an allocated (reserved) tray still fulfills the requirements for the source tray.
	 * This is the same as isLegalSourceTray except that the tray now should be reserved.
	 * @param tray The allocated tray to be checked
	 * @return Result of the check
	 */
	virtual bool checkAllocatedSourceTray(const auto_smart_factory::Tray& tray) const = 0;

	/**
	 * Checks if an allocated (reserved) tray still fulfills the requirements for the target tray.
	 * This is the same as isLegalTargetTray except that the tray now should be reserved.
	 * @param tray The allocated tray to be checked
	 * @return Result of the check
	 */
	virtual bool checkAllocatedTargetTray(const auto_smart_factory::Tray& tray) const = 0;

	/**
	 * Checks if the robot is suitable for this task, i.e. the package weight is
	 * lower (or equal) than the robot's maximum load.
	 * @param robotConfig Configuration of the robot
	 * @return True if robot is suitable
	 */
	bool isLegalRobot(const auto_smart_factory::RobotConfiguration& robotConfig) const;

	/**
	 * This is only used to determine which request to send to the agent.
	 *
	 * \todo It would be nicer to make also the request to the agent generic,
	 * i.e. independent of the type of input/output request.
	 *
	 * @return Depends on task type
	 */
	virtual bool getRobotRequestType() const = 0;

	/**
	 * Returns the package configuration associated to this task.
	 * @return Package configuration
	 */
	const auto_smart_factory::PackageConfiguration& getPackageConfig() const;

	/**
	 * Returns tray id of the fixed tray, i.e. the input tray for an input task
	 * and the output tray for an output task.
	 * @return Tray id
	 */
	virtual unsigned int getKnownTrayId() const = 0;

protected:
	/**
	 * Query the state of a specific tray.
	 * @param trayId Id of the tray
	 * @return Current tray state returned from storage management
	 */
	static auto_smart_factory::TrayState getTrayState(unsigned int trayId);

	/// The associated package configuration
	auto_smart_factory::PackageConfiguration pkgConfig;
};

typedef std::shared_ptr<TaskRequirements> TaskRequirementsPtr;
typedef std::shared_ptr<const TaskRequirements> TaskRequirementsConstPtr;

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TASKREQUIREMENTS_H_ */
