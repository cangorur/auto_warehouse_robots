Auto-Smart-Factory                         
============

by Orhan Can Görür* (orhan-can.goeruer@dai-labor.de), Jacob Seibert, Marc Schmidt, Malte Siemers, Utku Özmü, Ansgar Rösig, Hafiz Hamza, Paul Dieterich, Jonathan Regef, Mohannad Al Dakhil, Puriwat Khantiviriya.

We present a closed solution for a simulated autonomous smart factory.
The environment consists of two parts: production, warehousing. This project involves the warehousing part of it, but it is the development version. That is, some nodes are missing/left for the students to develop their own solutions (please refer to the architecture drawing below). The nodes to be adjusted are simply listed here:
* agent/Agent.cpp
* task_planner/Request.cpp
* charging_management/ChargingManagement.cpp
* path_planner/PathPlanner.cpp
* task_planner/TaskPlanner.cpp
* agent/ObstacleDetection.cpp
More information are given under their source files (headers). For simplicity you can build the project code documentation and see what is expected from you to develop on these nodes (please refer to 'Additional Tools' section)

Warehousing is an essential part of smart factories that tries to cope with the challenges of modern consumer behavior. The autonomous warehousing comes with a range of problems, i.e. Task Management (if not a part of overall planning), roadmap planning, finding feasible trajectories for and perception of the local environment of automated vehicles.
We show, that our solution scales well for warehouses of different sizes under mild constraints and in the number of robots.
We use a road map based approach for the planning of local and global paths and a semi-hierarchical model for task planning (to be integrated into overall planning).
The agent domain used for communication between the agents is ROS and we simulate the environments with MORSE.

An old documentation of the system describing the components and some services can be found under `doc/old_docs/`. NOTE: These documentations date back to the first design of the system, but most components are still there (slides are up-to-date).
For a new code documentation please refer to the section `Build code documentation` below.

---

## Prerequisites

### MORSE

Install Morse 1.3.1-STABLE. It is a forked project. Clone it from here:
```
https://github.com/cangorur/morse.git
```
Install python3-dev package
```
sudo apt install python3-dev
```
Then, create a build folder and cmake and install:
```
mkdir build && cd build
cmake -DBUILD_ROS_SUPPORT=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt ..
sudo make install
```
For the ROS interface of MORSE, follow the instructions here [link](http://www.openrobots.org/morse/doc/1.3/user/installation/mw/ros.html). But first read below and examine the steps in the link*!
- Step 1 and 2 under the link, you have already done.
- NOTE: For Step 3, 4 do the manual installation. That is, they have both apt-get install … OR from source. **Go with “from source” instructions**.
- Step 5 is already with one option.

### ROS

The code is tested with ROS Kinetic on Ubuntu 16.04 machines.
No special ROS packages are needed apart from those which come with a standard installation of ROS.

The only dependency for the installation is the *Boost libraries* (usually cpreinstalled with OS)
For running the python nodes, we have extra dependencies:
* numpy (sudo apt install python3-numpy, also install for python2: ROS is running that) --> you can install with pip,
* scipy (sudo apt install python3-scipy, also install for python2: ROS is running that) --> you can install with pip,
* networkx (sudo pip3 install networkx, also install for python2: sudo pip install networkx)

---

## Usage

### Installation

Before running the system make sure to build:

```
catkin_make install
```

And source the ros workspace automatically created after building the system:

```
source devel/setup.bash
```
Note that the devel folder is created after catkin_make under the project folder

### Running
The system can be launched using different environments, i.e. different smart factory designs.
We refer to this as different *factory configurations*.
These are configuration files stored in JSON format.

The following configurations are included as examples and are located in `configs`:

- smart_factory_config.json
- smart_factory_config_simple.json

You can simply delete/add from the .json file the robots, human (tested) and the trays (not tested) to change the system complexity if you have issues running with your PC. If you want to run different .json files, change these below:
* auto_factory.py line 9 (the main MORSE builder script which is used to launch the MORSE simulator)(see all builder scripts under `factory_morse/auto_smart_factory`). NOTE: Optionally create a new builder script with each config you prepare, see the examples under the given directory.
* full_system.launch line 33 (ros parameters) (see `src/auto_smart_factory/launch`)

Additionally, some other example builder scripts for large and simple warehouse configurations, or the same factory but with simple configuration (`smart_factory_config_simple.json`) are also available.
It is recommended that you run simple versions of the environments as below:
- Launch `full_system_simple.launch`,
- and run `auto_factory_simple.py` for a version without conveyor, human and pr2 robots, which runs significantly faster.
(NOTE: Optionally create new launcher with each config you prepare, see all the launch files under `src/auto_smart_factory/launch`).

Finally, robot and package configurations can be changed to embed specifications to the robots used in motion / task planning (e.g. package carrying capabilities, discharging / charging rates of the robots, see more under `configs`).

#### Running MORSE:

Make sure morse knows the simulation by importing it ONCE (from within the main directory):

```
cd factory_morse
morse import auto_smart_factory
```

Before starting the simulation also start a **roscore**. This is not a must (as roslaunch in the next step automatically runs it) but it allows you to freely navigate in the environment.
If a human model is added too, first press "F5" to switch the camera to world view.

You need to show your morse installation path and python libraries there to python env. variable:

```
echo 'export PYTHONPATH=${PYTHONPATH}:<morse_installation_path>/lib/python3/dist-packages' >> ~/.bashrc
source ~/.bashrc
```
TIP: Learn about .bashrc it is an essential part of the OS.

And yes, we need to source ros workspace again since MORSE uses the middleware:

```
source devel/setup.bash
```

Then you can start the environment by running

```
morse run auto_smart_factory auto_factory.py
```

**NOTE:** For lesser graphics and CPU requirements, you can run the simple version of the environment <auto_factory_simple.py>. In that case, please run <full_system_simple.launch> for the roslaunch below.
**NOTE-2:** For all the other maps (different builder scripts), run it corresponding roslaunch files (all named the same).

#### Running ROS:
For the included example warehouse configurations we provide also launch files that launch our system with the correct configuration files.

Apart from that you then have to adjust the full_system.launch file according to the warehouse configuration file to be used and the number and names of robots in the configuration file.
This is necessary because roslaunch does not allow to automatically create as many robot nodes as needed by the configuration.
Also specify the warehouse configuration file to be used in the parameters of the configuration server node.

Then you can launch the system (with MORSE already started) by running (DON'T FORGET TO SOURCE AGAIN)

```
roslaunch auto_smart_factory full_system.launch
```

### Hints for the Developers

#### For Task Planner Updates
Task Planner agent consists of multiple classes to ease the process.
- It all starts with the `TaskPlannerNode`, where it initializes the agent and creates an object of TaskPlanner
- TaskPlanner has the main operation. In general, `newInputRequest and newOutputRequest` are the service functions to start the main operation
- These are advertised services, and they are called currently by the `PackageGenerator` agent.
- So PackageGenerator creates a random package and input/output request, then it calls the TaskPlanner's `new_input_task`, `new_output_task` services. Feel free to change this random creation under `PackageGenerator.cpp`.
- Carefully examine these service functions under TaskPlanner.
- Under these functions, new `Request` objects are created for each request. Please examine the `Request.cpp`
- Requests are created with the `InputTaskRequirements` and `OutputTaskRequirements`. Feel free to edit these classes too, to define new properties and make the planning fun.
- You will notice under Request class that there is the function called `allocateResources`.
- This function first finds an available trays under `findTargetCandidates`, then a robot to assign under `getRobotCandidates`.
- These two functions are the core of your work. The strategies are to be developed under them.

#### For Motion and Path Planner Updates

- Task Planner assigns a task to the Agent by calling `assignTask()` function in the `Agent.cpp`. It only sends 2 information, ids of input tray and output tray.
- Then in `assignTask()`, each task is basically divided into 2 paths, one from current position to the source tray and the other from source tray to the target tray. For each path, it calculates several points, for example, drive point, approach point, drive back point, etc.
- Afterwards, it initiates a `Path` object (see Path.cpp) with all these parameters. So, basically for each task, two Path objects are initialized for the two separate paths mentioned above.
- The first path is set as `currentPath` and the second path is pushed onto the stack.
- Now `getTheNextPath()` is called which asks for a path from `PathPlanner.cpp` for the current `Path`. This asking is through the service, `request_new_path`, which implements `PathFinding` function under the PathPlanner code. This function is left for the students to develop their own solutions. You are free to use the roadmap generated by `RoadmapGenerator.py`.     
- Once the path is returned to the `Agent`, a `Plan` object is generated. `Plan` can be basically a chunk of the planned path or always the whole path. It depends on what you return from the PathPlanner,i.e. a whole path or a chunk of it. In the previous implementation, PathPlanner returns a chunk of the path and tells whether it’s the last chunk or not via a flag named `is_last_chunk`.If you want the PathPlanner to return the whole the first time, you can calculate and return the whole path from PathPlanner along with `is_last_chunk` set to true.
- The Agent then start driving this `Plan` by sending corresponding information to the `MotionPlanner`.
- On every call to `update()` function of Agent, which btw is called under the `poseCallback` or `gpsCallback` in another version, the Agent checks whether the current `Plan` is done or not. If not, it keeps on driving it
- If the current `Plan` is done (with the approach behavior also executed), then it checks whether the current `Path` is done. If not, then it asks for the next chunk (`Plan`) of the current `Path`. If the current `Path` is done (which is the first path to the source tray or to the charging station), there would be a `Load`, `Unload` or `Charge` taking place. See how those are arranged under `getTheNextPath()`.
- Finally, we pop out the next `Path` (if a package taken, then this would be to place to a target tray) from the stack that we pushed earlier and then the whole process is repeated.
- The `laserCallback` and `poseCallback` are the callback functions for the subscribed sensors. They trigger updates on `ObstacleDetection` and `MotionPlanner` objects created. To update a robot's driving behaviors, please refer to the `update` functions of both of the classes.
- `driveCurrentPath` will hold your driving strategies, and under ObstacleDetection `analyzeLaserScan` will hold your obstacle detection and avoiding strategies.

### ISSUES

This section will be extended with some more limitations and issues the current version holds:
* Tray numbering in the code is different than the labels written on the ground of the simulation environment. Even the charging stations are created from the trays, therefore they are all numbered together in the morse environment (see `factory_morse/auto_smart_factory/src/map_setup.py`). First charging stations (#0-7), inputs (#8-11), storages (#12-30, UPPER STORAGES ARE NOT NUMBERED and REGISTERED), outputs (#31-35).
* RELEVANT TO DEMO VERSION (NOT THIS ONE): Package generator, `src/auto_smart_factory/src/package_generator/PackageGenerator.cpp`, generates new pkgs on conveyor-1, which then always fall into input tray-1 (tray#8 in code numbering, see line 322 for the hard-coded x,y,z coordinates). The service `newPackageInputOnConveyor` in line 290 is the generator service and is directly connected to the websocket (see factory_gateway node), or can be called from terminal. For the other input trays, the packages are generated randomly by `newPackageInput` service see line 363 (it is only called in PackageGenerator.cpp).

### Additional Tools

#### Abstract visualization

For a deeper insight into the system an additional visualization can be viewed using rviz.
This visualization is launched by the following command:

```
roslaunch auto_smart_factory visualization.launch
```
#### WebSocket Interface

The sensors and actuators are open to the outside world (outside of morse and ros) through a websocket connection. The source code responsible for this is the "factory_gateway" ROS node. The node simply subscribes/requests the existing services for sensors and actuators on the system and sends all the information through websocket when requested (the node is the web service). By working on this node, you can simply add new services and define a .json file to introduce new interfaces to the system elements.

The node is NOT running by default (if you want you can add it to launch files). So run the node:

```
rosrun auto_smart_factory factory_gateway
```

Navigate to `src/auto_smart_factory/test_websocket` to test this connection easily (even from your PC, see more from the readme file there)

#### Record results

To analyze the results of the system, i.e. finished tasks, timings, robot states, the relevant messages can be saved to a bag file using **rosbag**.
The record can be started by launching rosbag with the prepared launch file (preferably before you start the system itself):

```
roslaunch auto_smart_factory log_results.launch
```

The results are saved to `logs/auto_smart_factory.bag`.
This bag file can then easily be analyzed using rosbag's Python/C++ API ([http://wiki.ros.org/rosbag/Code API](http://wiki.ros.org/rosbag/Code%20API)).

#### Test run the robots
The robots can be moved around for test purposes. Additionally, they can interact with packages. First we need to activate the test mode for the robot:
```
rosparam set /motion_planner_robot_<INTEGER_ID>/activate_tests true # Ex. /motion_planner_robot_2
```
This command will stop the robot even if it was following a given path. Once the parameters is set `false`, the robot automatically keeps following the previous track.
Then, we are able to control the robot motion with the following publisher command. This publishes to the robot motor model in the MORSE environment. The part of the command starting with the quotation marks are the argument of the publisher. You can press **tab** twice to retrieve the argument automatically. For the linear motion, we should always use only `X` axis and the value entered is the speed. For the angular motion please use only `Z` axis for CW or CCW rotation.
```
rostopic pub /robot_2/motion geometry_msgs/Twist
"linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
Additionally, we can activate a robot's gripper to grip (suck) or release a package.
To activate the gripper, the robot should be very close to the package. Use the following rosservice call
```
rosservice call /robot_2/gripper/load  # replace load with unload to release a package
```
In addition, in case it is needed somehow, one can move a package to anywhere on the world map.
```
# To move a package (as always, the commands between quotation marks are arguments and can be retrieved by doing double tab)
rosservice call /package_manipulator/move_package "package_id: '<STRING_ID_OF_A_PKG>'
x: 3.0
y: 5.0
z: 3.0"
# Examples of package IDs: pkg1_3, pkg2_8 . pkg1 is for light and pkg2 is for heavy packages.
```

#### Build code documentation

Documentation for the code can be generated using `rosdoc_lite` which we use basically as a wrapper of doxygen.
If you want to use `rosdoc_lite` install it from ROS repository.

To generate the documentation you can use a predefined CMAKE target by running:

```
catkin_make auto_smart_factory_doc
```

This most likely will put the documentation in your *build/auto_smart_factory/doc/html* directory of your workspace.
If you want to specify a different location you can run it yourself with:

```
rosdoc_lite -o <OUTPUT DIRECTORY> <PACKAGE PATH>
```

The mentioned *html* folder contains:

- **index.html**: Top-level page for the code documentation
- **index-msg.html**: Top-level page for message and service documentation
- **srv** folder under **html** holds the service documentations --> also see their advertised functions in their parent ROS nodes.

---

## Additional configuration files

The environment is defined by several configuration files. They are located in the `configs` directory and are stored in the JSON format.

- **warehouse configuration**: This defines number and locations of warehouse trays and robots.
- **robot configuration**: This defines properties of the different robot types (only one so far).
- **package configuration**: This defines properties of the different package types (only two so far).

For more information about the structure of these configuration files see [Configuration File Documentation](@ref configFiles).

### Warehouse Configs
These config files are called within the morse builder script and the roslaunch files. E.g. <auto_factory.py, auto_factory_walls.py>. Please run its corresponding roslaunch file to properly call out these configs and construct the map requested.
The naming of the warehouse config files has so far pointing out the complexity of the environment regarding the walls built around, storage amounts, additional robots etc.
The json file is very easy to track. One can simply:
- Add more storages, input and output trays, charging stations and more robots
- Change the locations of all trays (input, output, storage and charging)
- Add walls

### Package Configs

- **pkg1**: 25 x 25 x 25 cm, 10 kg, light
- **pkg2**: 25 x 25 x 25 cm, 50 kg, heavy
Note: The package weights and the robots carriage capacities are not currently considered. It is optional to take that into account in task planning.

### Robot Configs

- **Pioneer P3-DX**: Max. load = 50 kg, dark brown mark
- **Pioneer P3-DX Light**: Max. load = 25 kg, light brown mark
Note: The package weights and the robots carriage capacities are not currently considered. It is optional to take that into account in task planning.

---

## Brief overview of the system

System Architecture drawing of the new version of the project, also showing the nodes to be updated for the student developments
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/smart_factory_ws18_g3/raw/master/doc/system_architecture.png)

Advertised services/topics are listed below the corresponding node (THE LIST IS OLD, NEEDS TO BE UPDATED, see the ros service documentation folder under code documentation !!!).

- `config_server`: Provides services to get map configuration and robot configurations (package configurations will follow).
    - **Topic** `occupancy_map`: Publishes the occupancy map but is only meant to be used for visualization purposes.
    - **Service** `get_map_configuration`: Retrieves the warehouse configuration.
    - **Service** `get_robot_configurations`: Retrieves the robot configurations.
    - **Service** `get_package_configurations`: Retrieves the robot configurations.
- `package_generator`: Generates inputs & outputs. This simulates a context for the warehousing system.
    - **Service** `init`: Initializes the package generator.
- `package_manipulator`: Manipulates the position of packages.
    - **Service** `move_package`: Moves the given package to the given position. Expects the package ID and x,y,z positions.
- `storage_management`: Holds the current storage state of the warehouse. Tray sensors are connected to it and update the state.
    - **Topic** `storage_update`: Publishes the state of the corresponding tray every time a tray sensor detects changes.
    - **Service** `init`: Initializes the storage management.
    - **Service** `get_storage_information`: Returns the current storage state. Useful for initial call.
    - **Service** `reserve_tray`: Marks a storage tray as *reserved* (if possible, i.e. it is not occupied or already reserved).
    - **Service** `end_reservation`: Removes reservation from a storage tray.
    - **Service** `new_package_input`: Reserves input tray for a new incoming package.
    - **Service** `new_package_output`: Reserves output tray for a requested package.
    - **Service** `get_package`: Retrieves the package information stored for a specified tray.
    - **Service** `set_package`: Stores the package information for a specified tray.
    - **Service** `get_tray_state`: Returns the current state of a specific tray.
- `task_planner`: Plans and supervises the tasks.
    - **Topic** `status`: Publishes the current status of the task planner containing the states of all current requests and tasks.
    - **Service** `init`: Initializes the task planner.
    - **Service** `register_agent`: Registers agents.
    - **Service** `new_input_task`: Try to add new input request.
    - **Service** `new_output_task`: Try to add new output request.
- `agent`: Handles a robot. Can be launched using `rosrun auto_smart_factory agent robot_<number>`.
    - **Topic** `/robot/heartbeats`: Publishes regular status update messages from all robots.
    - **Topic** `robot_<number>/battery`: Publishes the current battery level of the robot.
    - **Topic** `robot_<number>/gripper_state`: Publishes load or unload events of the robot's gripper.
    - **Topic** `robot_<number>/laser_scanner`: Publishes the laser scanner sensors data.
    - **Topic** `robot_<number>/motion`: Publishes motion commands for the actuator.
    - **Topic** `robot_<number>/pose`: Publishes the robot's ground truth pose in space.
    - **Service** `robot_<number>/init`: Initializes the agent.
    - **Service** `robot_<number>/store_package`: Requests agent to calculate the plan to store a package.
    - **Service** `robot_<number>/retrieve_package`: Requests agent to calculate the plan to retrieve a package.
    - **Service** `robot_<number>/assign_task`: Requests agent to execute the task with the given id.
    - **Service** `robot_<number>/gripper/get_gripper_status`: Retrieves the current gripper status.
    - **Service** `robot_<number>/gripper/load`: Grabs a package if one is near.
    - **Service** `robot_<number>/gripper/unload`: Releases a grabbed package.
- `charging_management`: automatically tracks the battery levels of all the robots
    - **Topic** `/robot/heartbeats`: Publishes regular status update messages from all robots.
    - **Service** `get_free_charging_stations`: Returns the available charging stations.
    - **Service** `register_agent_charging_management`: Registers a robot agent to the management for battery tracking. Agent.cpp handles this at the start.
    - **Service** `robot_<number>/gripper/load`: Grabs a package if one is near.
    - **Service** `robot_<number>/gripper/unload`: Releases a grabbed package.
- `gripper_manipulator`: Manipulates the position of a robot's gripper.
    - **Service** `move_gripper`: Moves the gripper of a robot to a given position. Expects the gripper ID (same as robot IDs) and x,y,z positions
- `warehouse_management`: Initializes all system components & starts the simulation.
    - **Topic** `occupancy_map`: The occupancy map of the warehouse containing static obstacles. For visualization purposes.
    - **Topic** `abstract_visualization`: Visualization markers of the warehouse interior used for the rviz visualization.
- `roadmap_generator`:
    - **Topic** `roadmap_graph`: Publishing the roadmap generated.
    - **Service** `trigger_roadmap_generator`: Initializes and initiates the roadmap generation
- Additional MORSE topics
    - **Topic** `/warehouse/tray_sensors`: Each tray has a sensor to detect packages that are put into it. Changes are published to this topic.
