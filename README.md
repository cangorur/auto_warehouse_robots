Auto-Smart-Factory                         
============

A smart factory environment is an important aspect of "Industry 4.0" where products can be ordered, produced, sorted and stored in a warehouse to be finally delivered to the recipient.
We present a closed solution for a simulated autonomous warehousing. The work focuses on the transportation of products inside a warehouse by mobile robots. The goal is to implement a scalable system for cooperative online multi-agent task and path planning. Products and delivery requests arrive at an unknown time and frequency and need to be stored.
To handle a high demand of tasks, multiple robots have to coordinate their movement in a limited space and need to cooperate to minimize the overall time to deliver packages.

A hybrid task planning enables the cooperation between the robots. A central task planner is used to assign tasks to the agents based on their combined feedback. The robots individually evaluate their ability to fulfil a task (i.e., current workload, ETA, battery conditions) and report the result to the task planner which selects the most suitable agent for each task. This distributes the computational effort between the agents and the central instance. Once a task is received, each robot coordinates and negotiates with the other robots to dynamically plan their paths with less or no congestion based on a timed reservation system. This enables the decentralized path planning to consider both waiting and detours to find the fastest route. A motion planning ensures quick driving and accurate path following while considering all reservations. With this hybrid planning structure, the robots are able to dynamically plan their ways in a free-roaming environment with no certain lines to follow/cross. This provides a more efficient spacing in a factory and less distance to cover for each robot for the pickup and delivery. 

For now, we provided a system architecture showing the agents and the process flow on a coarse level [here](#brief-overview-of-the-system). The project's details are to be reported/published as a paper soon.

Demo videos:
- Multi-agent planning in free roaming env. (higher uncertainty): https://drive.google.com/file/d/1AYt4Dh8DFovb4Kjwdjfp571lD-n4fy7X/view?usp=sharing 
- Smart factory integration: https://drive.google.com/file/d/1QUMt3UhKa35j-W0HZqoHU2Gg8qAuC8GJ/view?usp=sharing

Here are the current branches and the descriptions:
1. `main`: free-roaming scenario where the robots are free to move to anywhere in the environment
2. `line-following`: we run the same system on a line-following (SoTA) warehouse system for comparison
3. `develop`: actively improving the free-roaming scenario to add more features: e.g., when a robot dies (dynamically handle its task, take the robot as a static obstacle), decentralized obstacle avoidance.
4. (to be added) `iot_gateway`: a gateway agent added to the system to interface (control and monitor) the entire warehouse system from outside of ROS (through our REST API). Documentation on the json formats to interface is provided under `src/auto_smart_factory/test_websocket/readme_socketInt.md`.
5. (to be added) `template`: a version of the system for the developers to easily integrate their own solutions to the environment.

---

## Prerequisites

### ROS

The code is tested with ROS Melodic and ROS Kinetic. We recommend using ROS Melodic. No special ROS packages are needed apart from those which come with a standard installation of ROS.

The only dependency for the installation is the *Boost libraries* (usually cpreinstalled with OS)
For running the python nodes, we have extra dependencies:
* numpy (sudo apt install python3-numpy, also install for python2: ROS is running that) --> you can install with pip,
* scipy (sudo apt install python3-scipy, also install for python2: ROS is running that) --> you can install with pip,
* networkx (sudo pip3 install networkx, also install for python2: sudo pip install networkx)

### MORSE

Install Morse 1.4: [link](https://github.com/morse-simulator/morse)

As Morse is not supported anymore, the version comes with package installation may complain about dependencies based on the OS. If you cannot configure properly, you can follow the manual instructions to install it from source with ROS dependency:
```bash
git clone https://github.com/morse-simulator/morse.git
cd morse && git checkout 1.4_STABLE
mkdir build && cd build
sudo apt install python3-dev #usually installed
cmake -DBUILD_ROS_SUPPORT=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt ..
sudo make install
sudo apt install blender # v2.79 is the latest version tested for our environment (see MORSE 1.4 blender support)
echo "export PATH=${PATH}:/opt/bin" >> ~/.bashrc # this is to make MORSE accessible everywhere
source ~/.bashrc
morse check # this shows the compatibility and that everything is installed fine
# now for ROS interface. Morse needs python3. Installing python3-yaml
wget http://pyyaml.org/download/pyyaml/PyYAML-3.10.tar.gz
tar xvf PyYAML-3.10.tar.gz
cd PyYAML-3.10
sudo python3 setup.py install
# roskpg for python3
wget https://bootstrap.pypa.io/ez_setup.py
sudo python3 ez_setup.py
git clone git://github.com/ros/rospkg.git
cd rospkg
sudo python3 setup.py install
# catkin for python3 support
git clone git://github.com/ros-infrastructure/catkin_pkg.git
cd catkin_pkg
sudo python3 setup.py install
cd .. && git clone git://github.com/ros/catkin.git
cd catkin
sudo python3 setup.py install
```

---
## Installation

Before running the system make sure to build:

```
catkin_make install
```

And source the ros workspace automatically created after building the system:

```
source devel/setup.bash
```
Note that the devel folder is created after catkin_make under the project folder

## Running

### Easy Run (tmux)
When tmux is installed (`sudo apt install tmux`) then 
```
./run.sh # sources and runs the whole environment (including the simulation)
```
### Manual Run:

Make sure morse knows the simulation by importing it ONCE (from within the main directory):

```
cd factory_morse
morse import auto_warehouse
```

Before starting the simulation also start a **roscore**. This allows you to freely navigate in the environment.

You need to show your morse installation path and python libraries there to python env. variable:

```
echo 'export PYTHONPATH=${PYTHONPATH}:<morse_installation_path>/lib/python3/dist-packages' >> ~/.bashrc
source ~/.bashrc
```

Source ros workspace again since MORSE uses the middleware:

```
source devel/setup.bash
```

Then you can start the simulation environment by running

```
morse run auto_warehouse auto_factory_paper.py
```

Running ROS:

```
source devel/setup.bash
roslaunch auto_smart_factory full_system_paper.launch
```

### Different Configurations
The system can be launched using different environments, i.e. different smart factory designs.
We refer to this as different *factory configurations*.
These are configuration files stored in JSON format.

The following configurations are included as examples and are located in `configs`, e.g., `smart_factory_config_paper.json` is the latest version we used for our experiments on multi-robot planning under uncertainty (e.g., free roaming environment).

More information on configs can be found [here](#additional-configuration-files).

You can simply delete/add from the .json file the robots, human (only supported in Morse 1.3 and ROS Kinetic) and the trays to change the system complexity if you have issues running with your PC. If you want to run different .json files, change these below:
* auto_factory.py line 9 (the main MORSE builder script which is used to launch the MORSE simulator)(see all builder scripts under `factory_morse/auto_warehouse`). NOTE: Optionally create a new builder script with each config you prepare, see the examples under the given directory.
* full_system.launch line 33 (ros parameters) (see `src/auto_smart_factory/launch`)

Additionally, some other example builder scripts for large and simple warehouse configurations, or the same factory but with simple configuration (`smart_factory_config_simple.json`) are also available.
It is recommended that you run simple versions of the environments as below:
- Launch `full_system_simple.launch`,
- and run `auto_factory_simple.py`

Finally, robot and package configurations can be changed to embed specifications to the robots used in motion / task planning (e.g. package carrying capabilities, discharging / charging rates of the robots, see more under the folder `configs`).


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

## Docker Instructions

We dockerized the project for wider use in our teaching activities. However, the project is an old version running in ROS Kinetic and the simulation performance is not optimized. We appreciate a new docker attempts.

```
docker run -it -p 5901:5901 -p 6901:6901 -v <full local path of this repo>:/headless/catkin_ws  shreyasgokhale/app-ras-smart-factory
```
1. Docker will pull the latest image from [Dockerhub](https://hub.docker.com/r/shreyasgokhale/app-ras-smart-factory) and mount your folder inside the container. The container already has VNC and noVNC installed. You can find out how container builds in [this Dockerfile](https://github.com/shreyasgokhale/APP-RAS-Docker/blob/master/Dockerfile). 
2. On command line, you can find out the URL where you can see the Xface UI of the container. For example: http://172.17.0.2:6901/vnc.html. Enter the password as ```vncpassword```.
3. Inside the container, execute ```sh /headless/catkin_ws/importProject.sh``` to import our project and set up paths.
4. Run the project by executing ```sh /headless/catkin_ws/run.sh```

---

## Hints for the Developers

### For Task Planner Updates
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

### For Motion and Path Planner Updates

- Then in `assignTask()`, each task is basically divided into 2 paths, one from current position to the source tray and the other from source tray to the target tray.
- A path query is initialized via the `ThetaStarPathPlanner` which returns a `Path` object (see Path.cpp) with all parameters. Theta* is used to find paths.
- Be careful, if no path was found a path object is still returned but is invalid. This MUST be check with path.isValid().
- This path is only used to calculate distances and estimated driving times. Paths that should be driven must be requested and registered with the Reservation Manager.
- When a Task is beeing executed the `TaskHandler` requests the next path from the Reservation Manager.

- The MotionPlanner gets the reserved path from the `TaskHandler` which is in charge of handing over the corret path and manually handle the approach and leave routines.
- On every call to `update()` function of Agent, which is called in the `poseCallback`, the MotionPlanner goes through a state machine to perform the appropriate action. By default it follows the path using a pid controller.
- When the current `Path` is finished, the MotionPlanner switches into the finished mode.

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

System Architecture drawing of the project:

![ngrok](https://github.com/cangorur/auto_warehouse_robots/blob/main/doc/architecture/system_architecture.png)



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

---

## Credits
- Copyright (c) is held by Orhan Can Görür and Technische Universität Berlin (TUB). 
- Copyright (c) 2017-2018 contributions are held by Orhan Can Görür, Jacob Seibert, Marc Schmidt, Malte Siemers, Utku Özmü, Ansgar Rösig, Hafiz Hamza, Paul Dieterich, Jonathan Regef, Mohannad Al Dakhil, Puriwat Khantiviriya. 
- Copyright (c) 2019-2021 contributions are held by Orhan Can Görür, Hafiz Hamza, Patrick Denzler, Florian Ziesche, Shreyas Gokhale, and Vincent Wölfer.
  
Below is the list of main contributors to the latest version:
- Orhan Can Görür (cangorur88@gmail.com, goeruer@tu-berlin.de)
- Hafiz Hamza (hafizhamza4013@gmail.com)
- Patrick Denzler
- Florian Ziesche
- Shreyas Gokhale
- Vincent Wölfer
