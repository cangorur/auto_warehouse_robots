
#include <include/agent/path_planning/ReservationBid.h>

#include "agent/path_planning/ReservationBid.h"

ReservationBid::ReservationBid(float bid, int agentId) :
	bid(bid),
	agentId(agentId)
{
}

ReservationBid::ReservationBid() :
	ReservationBid(-1.f, -1)
{
}
