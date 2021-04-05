# the unique tray ID
uint32 id

# Indicates whether the tray is occupied or not.
bool occupied

# Availability of the tray. If false the the tray is allocated by a task.
bool available

# information about the package stored in this tray (only valid if state != free)
Package package

