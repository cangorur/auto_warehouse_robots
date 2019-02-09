#ifndef AUTO_SMART_FACTORY_SRC_RESERVATION_MASTER_RESERVATIONMASTER_H_
#define AUTO_SMART_FACTORY_SRC_RESERVATION_MASTER_RESERVATIONMASTER_H_

#include "ros/ros.h"
#include <memory>
#include <thread>
#include <random>
#include <vector>
#include <exception>
#include <sstream>
#include <time.h>
#include "auto_smart_factory/ReservationRequest.h"

/**
 * The task planner component manages all incoming requests, checks for resources
 * and starts new tasks.
 */
class ReservationMaster {
public:
	explicit ReservationMaster();

	virtual ~ReservationMaster() = default;

	void update();

private:
	ros::Publisher reservationBroadcastPublisher;
	ros::Subscriber reservationRequestSubscriber;
	
	void reservationRequestCallback(const auto_smart_factory::ReservationRequest& msg);
	
	std::vector<auto_smart_factory::ReservationRequest> requests;

};


#endif /* AUTO_SMART_FACTORY_SRC_RESERVATION_MASTER_RESERVATIONMASTER_H_ */