# dimensions of the map in meters
float32 width
float32 height

# Info about the trays
Tray[] trays

# Info about the tray geometry
TrayGeometry tray_geometry

# The occupancy map of the warehouse
nav_msgs/OccupancyGrid occupancy_map

# The package pool
PackagePool package_pool

# Robots in this warehouse
Robot[] robots

# Idle positions of the robots
IdlePosition[] idle_positions

