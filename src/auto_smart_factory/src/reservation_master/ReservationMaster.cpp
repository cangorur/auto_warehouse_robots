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
		int highestRequestIndex = 0;
		
		for(int i = 0; i < requests.size(); i++) {
			if(requests[i].bid > highestBid) {
				highestBid = requests[i].bid;
				highestRequestIndex = i;
			}
		}		
		
		// Answer winner
		//ROS_INFO("[Reservation Master] Agent %d won auction", requests[highestRequestIndex].ownerId);
		auto_smart_factory::ReservationBroadcast broadcastReservationsMsg;
		broadcastReservationsMsg.isReservationBroadcastOrDenial = static_cast<unsigned char>(true);
		broadcastReservationsMsg.ownerId = requests[highestRequestIndex].ownerId;
		broadcastReservationsMsg.reservations = requests[highestRequestIndex].reservations;
		reservationBroadcastPublisher.publish(broadcastReservationsMsg);
		
		// Send deny to others
		for(int i = 0; i < requests.size(); i++) {
			if(i != highestRequestIndex) {
				//ROS_INFO("[Reservation Master] Agent %d lost auction", requests[i].ownerId);
				auto_smart_factory::ReservationBroadcast denyMsg;
				denyMsg.isReservationBroadcastOrDenial = static_cast<unsigned char>(false);
				denyMsg.ownerId = requests[i].ownerId;
				reservationBroadcastPublisher.publish(denyMsg);
			}
		}
	}
	
	requests.clear();
}

void ReservationMaster::reservationRequestCallback(const auto_smart_factory::ReservationRequest& msg) {
	requests.push_back(msg);	
}
