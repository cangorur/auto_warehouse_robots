# Configuration Files {#configFiles}

This document describes all configuration file structures in detail.

## Package Configuration

The package configuration file contains the properties of the different types a package can have.

Sample file:

```json
{
    "1" : {
        "width" : 0.25,
        "height" : 0.25,
        "weight" : 10.0
    },
    "2" : {
        "width" : 0.25,
        "height" : 0.25,
        "weight" : 50.0
    }
}
```

Each type is identified by a unique number and has three properties:

- **width**: Width of the package in meters. The package footprint is assumed to be quadratic.
- **height**: Height of the package in meters.
- **weight**: The weight of the package in kilograms.

## Robot Configuration

The robot configuration file contains information about the different robot types available.

Sample file:

```json
{
    "Pioneer P3-DX" : {
        "discharging_rate" : 0.05,
        "charging_rate" : 0.05,
        "motor_draining_rate": 0.5,
        "min_linear_vel" : 0.5,
        "max_linear_vel" : 1.5,
        "max_angular_vel" : 1.5,
        "radius" : 0.25,
        "max_load" : 50.0
    },
    "Kuka Jido" : {
        "discharging_rate" : 0.01,
        "charging_rate" : 3.0,
        "motor_draining_rate": 0.5,
        "min_linear_vel" : 0.5,
        "max_linear_vel" : 1.0,
        "max_angular_vel" : 1.0,
        "radius" : 0.4,
        "max_load" : 75.0
    }
}
```

Each robot configuration has an unique name and the following properties:

- **discharging_rate**: Battery discharging rate, in percent per seconds.
- **charging_rate**: Battery charging rate, in percent per seconds.
- **motor_draining_rate**: Battery discharging rate according to the motor activity (distance taken), in percent per seconds.
- **max_linear_vel**: Maximal linear velocity in meter per seconds.
- **max_angular_vel**: Maximal angular velocity in radians per second.
- **radius**: Radius of the circular footprint of the robot in meters. This should include the package if one is grabbed.
- **max_load**: Maximum load the robot can carry in kilograms.

## Warehouse Config

The warehouse configuration contains information about the general warehouse structure, e.g. tray and robot positions.

__Note:__ The environment Blender file only consists of a plane which serves as the ground of the warehouse. All trays, robots, packages and other objects are generated according to the warehouse configuration by the MORSE builder script.

### Coordinate System

The origin of the environment (world coordinates) is at the corner of the ground plane. The whole warehouse (except for the package pool) is by definition located in the first quadrant of the x/y coordinate system.

### File Structure

Sample file:

```json
{
    "map" : {
        "width" : 8,
        "height" : 8
    },
    "tray_geometry" : {
        "width" : 0.5,
        "height" : 0.5
    },
    "package_pool" : {
        "location" : {
            "x" : -5.0,
            "y" : 4.0,
            "z" : 0.0
        },
        "relative_drop_location" : {
            "x" : 0.0,
            "y" : -1.5,
            "z" : 2.0
        },
        "relative_stacking_area" : {
            "x1" : -2.25,
            "y1" : 0.0,
            "x2" : 2.25,
            "y2" : 2.25,
            "z" : 0.2
        }
    },
    "human_robot_collaboration": {
        "conveyors": [
        {
          "id": "conveyor_1",
          "type": "Object_Inspection",
          "location": {
            "x": 4.0,
            "y": 4.25,
            "z": 0.0
          },
          "rotation": 180
        },
        {
          "id": "conveyor_2",
          "type": "Object_Inspection",
          "location": {
            "x": 6.0,
            "y": 4.25,
            "z": 0.0
          },
          "rotation": 180
        }
      ],
        "humans": [
        {
          "id": "human_1",
          "expertise": "beginner",
          "mood": "stubborn",
          "position": {
            "x": "fixed to conveyor",
            "y": "fixed to conveyor",
            "z": 0.0
          },
          "orientation": 0
        },
        {
          "id": "human_2",
          "expertise": "beginner",
          "mood": "stubborn",
          "position": {
            "x": "fixed to conveyor",
            "y": "fixed to conveyor",
            "z": 0.0
          },
          "orientation": -180
        }
      ],
        "pr2_robots": [
        {
          "id": "pr2_2",
          "type": "proactive",
          "position": {
            "x": "fixed to conveyor",
            "y": "fixed to conveyor",
            "z": 0.0
           },
          "orientation": -90
        }
      ]
    },
    "trays" : [
        {
            "type" : "input",
            "x" : 0.5,
            "y" : 1.5,
            "orientation" : 0.0,
            "max_load" : 70.0
        },
        {
            "type" : "output",
            "x" : 7.5,
            "y" : 1.5,
            "orientation" : 180.0,
            "max_load" : 70.0
        },
        {
            "type" : "storage",
            "x" : 3.5,
            "y" : 4.5,
            "orientation" : 90.0,
            "max_load" : 70.0
        },
        {
            "type" : "storage",
            "x" : 3.5,
            "y" : 3.5,
            "orientation" : -90.0,
            "max_load" : 20.0
        },
    ],
    "robots" : [
        {
            "name" : "robot_1",
            "type" : "Pioneer P3-DX",
            "idle_position" : {
                "x" : 0.5,
                "y" : 0.5,
                "orientation" : 0.0
            }
        },
        {
            "name" : "robot_2",
            "type" : "Pioneer P3-DX",
            "idle_position" : {
                "x" : 0.5,
                "y" : 7.5,
                "orientation" : 0.0
            }
        }
    ],
    "kuka_robots" : [
	{
            "name" : "robot_kuka_1",
            "type" : "Kuka Jido",
            "idle_position" : {
                 "x" : 12.5,
                 "y" : 2.5,
		"orientation" : 180.0
            },
            "start_x" : 12.5,
            "start_y" : 2.5,
            "start_orientation" : 180.0
        },
	{
            "name" : "robot_kuka_2",
            "type" : "Kuka Jido",
            "idle_position" : {
                 "x" : 12.5,
                 "y" : 4.0,
		"orientation" : 180.0
            },
            "start_x" : 12.5,
            "start_y" : 4.0,
            "start_orientation" : 180.0
        }
     ]
}

```

The warehouse configuration has several sections which are explained below:

- *map*: This size of the warehouse map is used to generate the obstacle map and the roadmap. Therefore it should cover all areas where robots should be able to drive.
    - **width**: The width of the warehouse map in meters (from the world origin in positive x direction).
    - **height**: The height of the warehouse map in meters (from the world origin in positive y direction).
- *tray_geometry*: The rectangular footprint of a tray used to generate the obstacle map.
    - **width**: The width of a tray in x direction in meters.
    - **height**: The width of a tray in y direction in meters.
- *package_pool*: Location of the package pool and the drop point where unused packages can be deposited.
    - *location*
        - **x**: The x coordinate of the package pool center in meters.
        - **y**: The y coordinate of the package pool center in meters.
        - **z**: The z coordinate of the package pool center in meters.
    - *relative_drop_location*: Location of the drop point relative to the package pool location.
        - **x**: The relative x coordinate in meters.
        - **y**: The relative y coordinate in meters.
        - **z**: The relative z coordinate in meters.
    - *relative_stacking_area*: Location of the rectangular area where packages can be stacked initially relative to the package pool location. The rectangle is defined by two points p1 and p2.
        - **x1**: The relative x1 coordinate in meters.
        - **y1**: The relative y1 coordinate in meters.
        - **x2**: The relative x2 coordinate in meters.
        - **y2**: The relative y2 coordinate in meters.
        - **z**: The relative z height of the stacking area in meters.
- *trays*: Here the trays of the warehouse are defined. Individual IDs are generated at runtime. Each tray has the following properties:
    - **type**: The tray type. This can be either 'input', 'output' or 'storage'.
    - **x**: The x coordinate of the tray in meters. __Note:__ this is for the center of the tray.
    - **y**: The y coordinate of the tray in meters. __Note:__ this is for the center of the tray.
    - **orientation**: The orientation of the tray in degrees. __Note:__ For the sake of simplicity, currently only multiples of 90 degrees are supported!
    - **max_load**: The maximum load the tray can hold in kilograms.
- *robots*: Here available robots and their idle positions are defined.
    - **name**: Unique name/identifier of the robot. In our systems the scheme 'robot_*x*' is used.
    - **type**: The name of the corresponding robot configuration defined in the robot configuration file.
    - *idle_position*: The idle position is also the position where the robot starts.
        - **x**: The x coordinate of the idle position in meters.
        - **y**: The y coordinate of the idle position in meters.
        - **orientation**: The orientation of the idle pose in degrees.

