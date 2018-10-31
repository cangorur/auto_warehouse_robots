/*
 * TrayAllocator.h
 *
 *  Created on: 11.07.2017
 *      Author: jacob
 */

#ifndef AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TRAYALLOCATOR_H_
#define AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TRAYALLOCATOR_H_

#include <memory>

#include <auto_smart_factory/Package.h>

class TrayAllocator;
typedef std::shared_ptr<TrayAllocator> TrayAllocatorPtr;

/**
 * This class is a helper to manage reserving and de-reserving trays.
 * It ensures that reserved trays eventually will be de-reserved
 * (at the latest when the lifecycle of the TrayAllocator object ends).
 */
class TrayAllocator {
public:
	/**
	 * Creates a TrayAllocator object and returns a shared pointer to it.
	 * Shared pointers should be used to share the allocated tray between components.
	 * @param trayId The id of the tray to be allocated/reserved
	 * @return Pointer to the TrayAllocator object
	 */
	static TrayAllocatorPtr allocateTray(unsigned int trayId);

	/**
	 * Creates a TrayAllocator object and tries to reserve specified tray.
	 * The valid flag is set according to the success of reservation.
	 * @param trayId Specified tray
	 */
	TrayAllocator(unsigned int trayId);

	/**
	 * Destroys TrayAlocator object.
	 *
	 * \note If the reservation is valid, it is automatically ended on destruction.
	 */
	virtual ~TrayAllocator();

	/**
	 * Get the state of the valid flag.
	 * @return Valid flag
	 */
	bool isValid() const;

	/**
	 * Get id of associated tray.
	 * @return Tray id
	 */
	unsigned int getId() const;

	/**
	 * Set package information at the allocated tray.
	 *
	 * \todo Maybe this should be only allowed when valid is true.
	 *
	 * @param pkg Package information (id and type)
	 * @return Success of the storage management service call
	 */
	bool setPackage(const auto_smart_factory::Package &pkg);

	/**
	 * Get package information of the allocated tray.
	 *
	 * \todo Maybe this should be only allowed when valid is true.
	 *
	 * @return Package information (id and type). If storage management service call failed, id and type are 0.
	 */
	auto_smart_factory::Package getPackage();

private:
	/**
	 * Id of the associated tray.
	 */
	unsigned int trayId;

	/**
	 * Valid flag indicates if the tray is successfully allocated.
	 */
	bool valid;
};

#endif /* AUTO_SMART_FACTORY_SRC_TASK_PLANNER_TRAYALLOCATOR_H_ */
