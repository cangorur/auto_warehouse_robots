Auto-Smart-Factory                         
============

by Orhan Can Görür* (cangorur88@gmail.com), Hafiz Hamza, Jacob Seibert, Marc Schmidt, Malte Siemers, Utku Özmü, Ansgar Rösig, Paul Dieterich, Jonathan Regef, Mohannad Al Dakhil, Puriwat Khantiviriya.

Extended with decentralized multi-robot coordination and collaboration by Patrick Denzler, Florian Ziesche, Shreyas Gokhale, Vincent Wölfer, and Uros Petkovic.

A smart factory environment is an important aspect of "Industry 4.0" where products can be ordered, produced, sorted and stored in a warehouse to be finally delivered to the recipient. The different components of the scenario are modular and hence the information exchange between them is essential.
This work focuses on the transportation of products inside a warehouse. The goal is to implement a scalable system for cooperative multi-agent task and path planning. Products arrive unpredictably and need to be stored until they can be delivered to the customer.
To handle a high demand of tasks, multiple robots have to coordinate their movement in a limited space and need to cooperate to minimize the overall time to deliver packages.
A hybrid task planning enables the cooperation between the robots. A central task planner is used to assign tasks to the agents based on their combined feedback. The robots individually evaluate their ability to fulfil a task and report the result to the task planner which selects the most suitable agent for each task. This distributes the computational effort between the agents and the central instance.
Furthermore, the robots coordinate their movements to avoid collisions and congestion based on a timed reservation system. Each robot reserves its path on a shared map using estimated driving times. This enables the decentralized path planning to consider both, waiting and detours to find the fastest route.
A specialist motion planning ensures quick driving and accurate path following while considering all reservations.


---

## Docker Instructions

1. Install docker engine (CE) by following the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-engine---community).
2. Run follwing command on your terminal:

```
docker run -it -p 5901:5901 -p 6901:6901 -v <full local path of this repo>:/headless/catkin_ws  shreyasgokhale/app-ras-smart-factory
```

3. Docker will pull the latest image from [Dockerhub](https://hub.docker.com/r/shreyasgokhale/app-ras-smart-factory) and mount your folder inside the container. The container already has VNC and noVNC installed. You can find out how container builds in [this Dockerfile](https://github.com/shreyasgokhale/APP-RAS-Docker/blob/master/Dockerfile). 
4. On command line, you can find out the URL where you can see the Xface UI of the container. For example: http://172.17.0.2:6901/vnc.html. Enter the password as ```vncpassword```.
5. Inside the container, execute ```sh /headless/catkin_ws/importProject.sh``` to import our project and set up paths.
6. Run the project by executing ```sh /headless/catkin_ws/run.sh```


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
* auto_factory.py line 9 (the main MORSE builder script which is used to launch the MORSE simulator)(see all builder scripts under `factory_morse/auto_warehouse`). NOTE: Optionally create a new builder script with each config you prepare, see the examples under the given directory.
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
morse import auto_warehouse
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
morse run auto_warehouse auto_factory.py
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

#### Running via run script (tmux)
When tmux is installed (`sudo apt install tmux`) the run.sh script (`./run.sh`) runs the whole environment. It can be exited by pressing `CTRL+B` followed by `&` (`SHIFT+6`). With `CTRL+B` and the arrow keys the active tmux split can be selected and `CTRL+B` and `[` enables scrolling. Pressing `q` exists scroll mode. Google for `tmux cheatsheet` for more information on how to use tmux.


### Hints for the Developers

#### For Task Planner Updates
Task Planner agent consists of multiple classes to ease the process.
- It all starts with the `TaskPlannerNode`, where it initializes the agent and creates an object of TaskPlanner
- TaskPlanner has the main operation. In general, `newInputRequest and newOutputRequest` are the service functions to start the main operation
- These are advertised services, and they are called currently by the `PackageGenerator` agent.
- So PackageGenerator creates a random package and input/output request, then it calls the TaskPlanner's `new_input_task`, `new_output_task` services. Feel free to change this random creation under `PackageGenerator.cpp`.
- New `Request` objects are created for each request. Examine the `Request.cpp`
- Requests are created with the `InputTaskRequirements` and `OutputTaskRequirements`. Feel free to edit these classes too, to define new properties and make the planning fun.
- You will notice under Request class that there is the function called `allocateResources`.
- This function first finds an available trays under `findTargetCandidates`, then a robot to assign under `getRobotCandidates`.
- The `getRobotCandidates` function will publish a `taskAnnouncement` message on the `task_broadcast` topic to which the `TaskHandlers` in the agents will answer with a `TaskRating` message on the `task_response` topic. 
- The Task Planner will then sort the answers according to their score and assign the task to the best robot

#### For Motion and Path Planner Updates

- Then in `assignTask()`, each task is basically divided into 2 paths, one from current position to the source tray and the other from source tray to the target tray.
- A path query is initialized via the `ThetaStarPathPlanner` which returns a `Path` object (see Path.cpp) with all parameters. Theta* is used to find paths.
- Be careful, if no path was found a path object is still returned but is invalid. This MUST be check with path.isValid().
- This path is only used to calculate distances and estimated driving times. Paths that should be driven must be requested and registered with the Reservation Manager.
- When a Task is beeing executed the `TaskHandler` requests the next path from the Reservation Manager.

- The MotionPlanner gets the reserved path from the `TaskHandler` which is in charge of handing over the corret path and manually handle the approach and leave routines.
- On every call to `update()` function of Agent, which is called in the `poseCallback`, the MotionPlanner goes through a state machine to perform the appropriate action. By default it follows the path using a pid controller.
- When the current `Path` is finished, the MotionPlanner switches into the finished mode.


### Additional Tools

#### Abstract visualization

For a deeper insight into the system an additional visualization can be viewed using rviz.
This visualization is launched by the following command:

```
roslaunch auto_smart_factory visualization.launch
```

#### Record results

The system is evaluated with the Evaluator node. It can be configured in its respective header file.

The results are saved to the home directory.
This file can be imported into any spreadsheet or matlab tool as it is a simple csv file. Data samples are automatically taken during the simulation. Information is displayed in the terminal.


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
![ngrok](https://gitlab.tubit.tu-berlin.de/app-ras/smart_factory_ws14_g3/raw/master/doc/architecture/system_architecture.png)

Advertised services/topics are listed below the corresponding node (THE LIST IS OLD, NEEDS TO BE UPDATED, see the ros service documentation folder under code documentation !!!).

- `config_server`: Provides services to get map configuration and robot configurations (package configurations will follow).
    - **Service** `get_map_configuration`: Retrieves the warehouse configuration.
    - **Service** `get_robot_configurations`: Retrieves the robot configurations.
    - **Service** `get_package_configurations`: Retrieves the package configurations.
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
    - **Topic** `task_broadcast` : Announce tasks to the agents.
    - **Topic** `task_rating` : Collect answers to announcements.
    - **Service** `init`: Initializes the task planner.
    - **Service** `register_agent`: Registers agents.
    - **Service** `new_input_task`: Try to add new input request.
    - **Service** `new_output_task`: Try to add new output request.
- `agent`: Handles a robot. Can be launched using `rosrun auto_smart_factory agent robot_<number>`.
    - **Topic** `task_broadcast` : Receive task announcements.
    - **Topic** `task_rating` : Publish ratings for task announcements.
    - **Topic** `/robot/heartbeats`: Publishes regular status update messages from all robots.
    - **Topic** `robot_<number>/battery`: Publishes the current battery level of the robot.
    - **Topic** `robot_<number>/gripper_state`: Publishes load or unload events of the robot's gripper.
    - **Topic** `robot_<number>/laser_scanner`: Publishes the laser scanner sensors data.
    - **Topic** `robot_<number>/motion`: Publishes motion commands for the actuator.
    - **Topic** `robot_<number>/pose`: Publishes the robot's ground truth pose in space.
    - **Service** `robot_<number>/init`: Initializes the agent.
    - **Service** `robot_<number>/assign_task`: Requests agent to execute the task with the given id.
    - **Service** `robot_<number>/gripper/get_gripper_status`: Retrieves the current gripper status.
    - **Service** `robot_<number>/gripper/load`: Grabs a package if one is near.
    - **Service** `robot_<number>/gripper/unload`: Releases a grabbed package.
- `reservation_manager`: handle the timed reservations on the dynmaic map
    - **Topic** `/reservation_request`: Request turn to add reservation at reservation arbiter
    - **Topic** `/reservation_broadcast`: Broadcasts the reservations among all robots

- `charging_management`: automatically tracks the battery levels of all the robots
    - **Topic** `/robot/heartbeats`: Publishes regular status update messages from all robots.
    - **Service** `robot_<number>/gripper/load`: Grabs a package if one is near.
    - **Service** `robot_<number>/gripper/unload`: Releases a grabbed package.
- `gripper_manipulator`: Manipulates the position of a robot's gripper.
    - **Service** `move_gripper`: Moves the gripper of a robot to a given position. Expects the gripper ID (same as robot IDs) and x,y,z positions
- `warehouse_management`: Initializes all system components & starts the simulation.
    - **Topic** `abstract_visualization`: Visualization markers of the warehouse interior used for the rviz visualization.
    - **Topic** `robot_<number>_visualization`: Visualization markers of the individual robots like paths and reservations.
- Additional MORSE topics
    - **Topic** `/warehouse/tray_sensors`: Each tray has a sensor to detect packages that are put into it. Changes are published to this topic.
