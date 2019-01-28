# General content
float64 timestamp
int32 auctionId
int32 robotId

bool isReservationMessage

# Only if reservationMessage
Rectangle[] reservations

# Only if not reservationMessage
float32 bid