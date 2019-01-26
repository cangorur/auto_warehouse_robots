cmake_minimum_required(VERSION 2.8.3)
project(auto_smart_factory)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		nav_msgs
		roscpp
		roslib
		rospy
		sensor_msgs
		std_msgs
		message_generation
		tf
		)

## System dependencies are found with CMake's conventions
set(BOOST_COMPONENTS
		system
		coroutine
		context
		thread
		graph
		)
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

if (APPLE)
	set(OPENSSL_ROOT_DIR "/usr/local/opt/openssl")
endif ()

find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

find_package(Threads REQUIRED)
include_directories(. ${THREEADS_INCLUDE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
		FILES
		TrayGeometry.msg
		Tray.msg
		Package.msg
		RequestStatus.msg
		TaskState.msg
		TaskPlannerState.msg
		TaskAnnouncement.msg
		TaskRating.msg
		TraySensor.msg
		PackagePool.msg
		Robot.msg
		IdlePosition.msg
		WarehouseConfiguration.msg
		RobotConfiguration.msg
		RobotHeartbeat.msg
		AdditionalTime.msg
		PackageConfiguration.msg
		TrayState.msg
		StorageState.msg
		StorageUpdate.msg
		GripperState.msg
		CollisionAction.msg
		MapConfiguration.msg
		Rectangle.msg
)

## Generate services in the 'srv' folder
add_service_files(
		FILES
		InitAgent.srv
		InitTaskPlanner.srv
		InitPathPlanner.srv
		InitStorageManagement.srv
		InitPackageGenerator.srv
		#InitChargingManagement.srv
		RegisterAgent.srv
		StorePackage.srv
		RetrievePackage.srv
		AssignTask.srv
		GetWarehouseConfig.srv
		GetRobotConfigurations.srv
		GetPackageConfigurations.srv
		GetStorageState.srv
		GetTrayState.srv
		ReserveStorageTray.srv
		GetPackage.srv
		SetPackage.srv
		NewPackageGenerator.srv
		NewPackageInput.srv
		NewPackageOutput.srv
		MovePackage.srv
		MoveGripper.srv
		outputRequest.srv
		inputRequest.srv
		PerformTaskTest.srv
		SetConveyorSpeed.srv
		RotateTable.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
		DEPENDENCIES
		geometry_msgs
		nav_msgs
		sensor_msgs
		std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
		INCLUDE_DIRS include
		#LIBRARIES auto_smart_factory
		CATKIN_DEPENDS geometry_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs message_runtime
		#DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
		include
		${Boost_INCLUDE_DIR}
		${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/auto_smart_factory.cpp
# )

add_library(tray_allocation
		src/storage_management/TrayAllocator.cpp
		)
add_dependencies(tray_allocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tray_allocation
		${catkin_LIBRARIES}
		)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/auto_smart_factory_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# Warehouse Management
add_executable(warehouse_management_node
		src/warehouse_management/WarehouseManagementNode.cpp
		src/warehouse_management/WarehouseManagement.cpp)
set_target_properties(warehouse_management_node PROPERTIES OUTPUT_NAME warehouse_management PREFIX "")
add_dependencies(warehouse_management_node auto_smart_factory_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(warehouse_management_node ${catkin_LIBRARIES})

# Agent
add_executable(agent_node
		src/agent/path_planning/GridNode.cpp
		src/agent/path_planning/Map.cpp
		src/agent/path_planning/OrientedPoint.cpp
		src/agent/path_planning/Path.cpp
		src/agent/path_planning/Point.cpp
		src/agent/path_planning/Rectangle.cpp
		src/agent/path_planning/ThetaStarGridNodeInformation.cpp
		src/agent/path_planning/ThetaStarMap.cpp
		src/agent/path_planning/ThetaStarPathPlanner.cpp
		src/agent/path_planning/RobotHardwareProfile.cpp
		src/agent/path_planning/TimedLineOfSightResult.cpp

		src/agent/Agent.cpp
		src/agent/AgentNode.cpp
		src/agent/Gripper.cpp
		src/agent/MotionPlanner.cpp
		src/agent/ObstacleDetection.cpp		
		src/agent/Position.cpp

		src/agent/ChargingManagement.cpp

		src/agent/task_handling/TaskHandler.cpp
		src/agent/task_handling/Task.cpp
		src/agent/task_handling/TransportationTask.cpp
		src/agent/task_handling/ChargingTask.cpp
		
		src/agent/PidController.cpp

		src/Math.cpp
		)
set_target_properties(agent_node PROPERTIES OUTPUT_NAME agent PREFIX "")
add_dependencies(agent_node auto_smart_factory_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(agent_node ${catkin_LIBRARIES})

# Package Generator
add_executable(package_generator_node
		src/package_generator/PackageGenerator.cpp
		src/package_generator/PackageGeneratorNode.cpp)
set_target_properties(package_generator_node PROPERTIES OUTPUT_NAME package_generator PREFIX "")
add_dependencies(package_generator_node auto_smart_factory_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(package_generator_node
		tray_allocation
		${catkin_LIBRARIES}
		)

# Map Server
add_executable(config_server_node
		src/config_server/ConfigServerNode.cpp
		src/config_server/MapConfigServer.cpp
		src/config_server/RobotConfigServer.cpp
		src/config_server/PackageConfigServer.cpp
		)
set_target_properties(config_server_node PROPERTIES OUTPUT_NAME config_server PREFIX "")
add_dependencies(config_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(config_server_node ${catkin_LIBRARIES})

# Storage Management
add_executable(storage_management_node
		src/storage_management/StorageManagementNode.cpp
		src/storage_management/StorageManagement.cpp
		)
set_target_properties(storage_management_node PROPERTIES OUTPUT_NAME storage_management PREFIX "")
add_dependencies(storage_management_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(storage_management_node ${catkin_LIBRARIES})

# Charging Management
#add_executable(charging_management_node
#		src/Agent/ChargingManagementNode.cpp
#		src/Agent/ChargingManagement.cpp
#		)
#set_target_properties(charging_management_node PROPERTIES OUTPUT_NAME charging_management PREFIX "")
#add_dependencies(charging_management_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(charging_management_node ${catkin_LIBRARIES})


# cpptest
#add_executable(test_service_node
#    src/path_planning/testService.cpp
#    src/path_planning/testServiceNode.cpp
#)

#set_target_properties(test_service_node PROPERTIES OUTPUT_NAME test_service PREFIX "")
#add_dependencies(test_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_service_node ${catkin_LIBRARIES})

# Task Planner
add_executable(task_planner_node
		src/task_planner/TaskPlannerNode.cpp
		src/task_planner/TaskPlanner.cpp
		src/task_planner/RobotCandidate.cpp
		src/task_planner/Request.cpp
		src/task_planner/Task.cpp
		src/task_planner/TaskData.cpp
		src/task_planner/TaskRequirements.cpp
		src/task_planner/InputTaskRequirements.cpp
		src/task_planner/OutputTaskRequirements.cpp
		)
set_target_properties(task_planner_node PROPERTIES OUTPUT_NAME task_planner PREFIX "")
add_dependencies(task_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_planner_node
		tray_allocation
		${catkin_LIBRARIES}
		)

add_custom_target(${PROJECT_NAME}_doc
		rosdoc_lite ${PROJECT_SOURCE_DIR}
		)

# Factory Gateway
add_executable(factory_gateway_node
		src/factory_gateway/FactoryGatewayNode.cpp
		src/factory_gateway/FactoryGateway.cpp
		)
set_target_properties(factory_gateway_node PROPERTIES OUTPUT_NAME factory_gateway PREFIX "")
add_dependencies(factory_gateway_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(factory_gateway_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENSSL_CRYPTO_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
# Please add your new python nodes to here as a catkin_install
catkin_install_python(PROGRAMS
		src/package_generator/PackageManipulator.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

catkin_install_python(PROGRAMS
		src/agent/GripperManipulator.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

#catkin_install_python(PROGRAMS
#		src/roadmap_generator/RoadmapGenerator.py
#		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#		)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

install(DIRECTORY include include/agent include/config_server include/factory_gateway include/package_generator include/storage_management include/task_planner include/warehouse_management
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		PATTERN ".git" EXCLUDE
		)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_auto_smart_factory.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
