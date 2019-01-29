#ifndef PROJECT_RESERVATIONBID_H
#define PROJECT_RESERVATIONBID_H

class ReservationBid {
public:
	ReservationBid();
	ReservationBid(float bid, int agentId);

	float bid;
	int agentId;	
};

#endif //PROJECT_RESERVATIONBID_H
