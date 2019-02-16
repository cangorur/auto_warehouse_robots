
#include <include/reservation_master/ReservationMaster.h>

#include "reservation_master/ReservationMaster.h"
#include "auto_smart_factory/ReservationBroadcast.h"

ReservationMaster::ReservationMaster() {
	ros::NodeHandle pn("~");
	
	reservationBroadcastPublisher = pn.advertise<auto_smart_factory::ReservationBroadcast>("/reservation_broadcast", 100, true);
	reservationRequestSubscriber = pn.subscribe("/reservation_request", 100, &ReservationMaster::reservationRequestCallback, this);
}

void ReservationMaster::update() {
	if(!requests.empty()) {
		double highestBid = -1.f;
		int highestRequestIndex = -1;
		std::vector<int> emergencyStopRequests;
		
		for(int i = 0; i < requests.size(); i++) {
			if(requests[i].isEmergencyStop) {
				emergencyStopRequests.push_back(i);
				continue;
			}
			
			if(requests[i].bid > highestBid) {
				highestBid = requests[i].bid;
				highestRequestIndex = i;
			}
		}
		
		if(!emergencyStopRequests.empty()) {
			for(int i = 0; i < requests.size(); i++) {
				if(std::find(emergencyStopRequests.begin(), emergencyStopRequests.end(), i) != emergencyStopRequests.end()) {
					sendEmergencyStopBroadcastMessage(i);
				} else {
					sendDenyMessage(i);		
				}
			}			
		} else {
			sendReservationBroadcastMessage(highestRequestIndex);

			// Send deny to others
			for(int i = 0; i < requests.size(); i++) {
				if(i != highestRequestIndex) {
					sendDenyMessage(i);
				}
			}
		}

		requests.clear();
	}	
}

void ReservationMaster::reservationRequestCallback(const auto_smart_factory::ReservationRequest& msg) {
	requests.push_back(msg);	
}

void ReservationMaster::sendDenyMessage(int requestIndex) {
	//ROS_INFO("[Reservation Master] Agent %d lost auction", requests[i].ownerId);
	auto_smart_factory::ReservationBroadcast msg;
	msg.isReservationBroadcastOrDenial = static_cast<unsigned char>(false);
	msg.ownerId = requests[requestIndex].ownerId;
	reservationBroadcastPublisher.publish(msg);
}

void ReservationMaster::sendReservationBroadcastMessage(int requestIndex) {
	//ROS_INFO("[Reservation Master] Agent %d won auction", requests[highestRequestIndex].ownerId);
	auto_smart_factory::ReservationBroadcast msg;
	msg.isReservationBroadcastOrDenial = static_cast<unsigned char>(true);
	msg.isEmergencyStop = static_cast<unsigned char>(false);
	msg.ownerId = requests[requestIndex].ownerId;
	msg.reservations = requests[requestIndex].reservations;
	reservationBroadcastPublisher.publish(msg);
}

void ReservationMaster::sendEmergencyStopBroadcastMessage(int requestIndex) {
	ROS_INFO("[Reservation Master] Sending emergency stop for agent %d", requests[requestIndex].ownerId);
	auto_smart_factory::ReservationBroadcast msg;
	msg.isReservationBroadcastOrDenial = static_cast<unsigned char>(true);
	msg.isEmergencyStop = static_cast<unsigned char>(true);
	msg.ownerId = requests[requestIndex].ownerId;
	msg.reservations = requests[requestIndex].reservations;
	reservationBroadcastPublisher.publish(msg);
}
