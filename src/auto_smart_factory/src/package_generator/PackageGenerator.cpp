#include <package_generator/PackageGenerator.h>

PackageGenerator::PackageGenerator() {
	ros::NodeHandle pn("~");
	initSrv = pn.advertiseService("init", &PackageGenerator::init, this);
	// this service is to generate new packages by request. It also calls itself here under this node every certain time interval. Check heartbeatPeriod variable under the header
	generateNewPackageServer = pn.advertiseService("new_package_generator", &PackageGenerator::generateService, this);
}

PackageGenerator::~PackageGenerator() {
}

void PackageGenerator::update() {
	if(initialized) {
		if(!hasStorageState) {
			getStorageInformation();
		} else {
			if(!generating && isTimeForGeneration()) {
				generating = true;
				generate();
				generating = false;
			}
		}
	}
}

bool PackageGenerator::init(
		auto_smart_factory::InitPackageGenerator::Request& req,
		auto_smart_factory::InitPackageGenerator::Response& res) {
	if(!initialized) {
		initialized = initialize(req.warehouse_configuration, req.package_configurations);
		if(initialized)
			ROS_INFO("PackageGenerator succesfully initialized!");
		else
			ROS_ERROR("PackageGenerator initialization failed!");
	} else
		ROS_WARN("PackageGenerator has already been initialized!");
	res.success = initialized;
	return true;
}

bool PackageGenerator::initialize(
		auto_smart_factory::WarehouseConfiguration warehouse_configuration,
		std::vector<auto_smart_factory::PackageConfiguration> package_configurations) {

	warehouseConfig = warehouse_configuration;
	packageConfigs = package_configurations;

	//setup external packages datastructure
	initializePackages();

	//setup request plan
	if(doRequestPlans) {
		initializeRequestPlan();
	}

	//initialize random seed for rand()
	srand(time(NULL));

	storageUpdateSub = n.subscribe("storage_management/storage_update", 1000,
	                               &PackageGenerator::updateTrayState, this);
	trayStateClient = n.serviceClient<auto_smart_factory::GetTrayState>(
			"storage_management/get_tray_state");
	return true;
}

bool PackageGenerator::generateService(auto_smart_factory::NewPackageGenerator::Request& req,
                                       auto_smart_factory::NewPackageGenerator::Response& res) {
	bool input_chosen;

	ROS_INFO("[package generator] A Manual package request was received from conveyor belt!");

	std::vector<auto_smart_factory::TrayState> free_inputs = getFreeStorages("input");
	std::vector<auto_smart_factory::TrayState> free_outputs = getFreeStorages("output");
	std::vector<auto_smart_factory::TrayState> selected;

	// choose whether new input or output randomly: HERE INPUT MEANS TAKE THEM AND STORE? 
	if(req.tray.type == "input" && free_inputs.size() > 0) {
		input_chosen = true;
		selected = free_inputs;
	} else if(req.tray.type == "output" && free_outputs.size() > 0) {
		input_chosen = false;
		selected = free_outputs;
	} else {
		res.success = false;
		ROS_INFO("[package generator] No tray available for the selected operation!");
		return false;
	}

	for(int i = 0; i < selected.size(); i++) {
		if(selected[i].id == req.tray.id) {
			break; // means the requested tray is available
		} else if(i == (selected.size() - 1)) {
			res.success = false;
			ROS_INFO("[package generator] The requested tray is not available: Type: %s || ID: %d", input_chosen ? "Input" : "Output", req.tray.id);
			return false;
		}
	}

	auto_smart_factory::Tray tray;
	tray = getTray(
			req.tray.id); // TODO: here will be updated to get the trays not only through their inputs but also their types !
	if((tray.package_type != 0) && (tray.package_type != req.package.type_id)) {
		res.success = false;
		ROS_INFO("[package generator] The requested tray: %d does not support package type: %d", tray.id, req.package.type_id);
		return false;
	}

	ROS_INFO("[package generator] Next package generation: %s!", input_chosen ? "Input" : "Output");
	bool success = false;

	// package.type_id = packageConfigs[index].id;
	auto_smart_factory::Package package = req.package;
	if(input_chosen) {
		package.id = externalPackages[req.package.type_id - 1].front();
		success = newPackageInputOnConveyor(tray, package);
		if(success) {
			externalPackages[req.package.type_id - 1].pop_front();
		}
	} else {
		success = newPackageOutput(tray.id, package);
	}

	if(success) {
		updateTimer();
		ROS_INFO("[package generator] Package has been generated !");
	}
	res.success = success;
	return success;
}


void PackageGenerator::generate() {
	bool input_chosen;
	auto_smart_factory::Tray tray;

	if(doRequestPlans && requestPlans.empty()) {
		ROS_INFO("[package generator] Plan doesn't contain any requests anymore! ");
		return;
	}

	std::vector<auto_smart_factory::TrayState> free_inputs = getFreeStorages("input");
	std::vector<auto_smart_factory::TrayState> free_outputs = getFreeStorages("output");
	std::vector<auto_smart_factory::TrayState> selected;

	// choose whether new input or output randomly
	if(free_inputs.size() > 0 && free_outputs.size() > 0) {
		if(doRequestPlans) {
			input_chosen = requestPlans.front().first == "input";
		} else {
			int r = rand() % 5;
			input_chosen = (r == 0 || r == 1 || r == 2 || r == 3) ? true : false; // 80% it is an input request
		}
		selected = input_chosen ? free_inputs : free_outputs;
	} else if(!doRequestPlans && free_inputs.size() > 0) {
		input_chosen = true;
		selected = free_inputs;
	} else if(!doRequestPlans && free_outputs.size() > 0) {
		input_chosen = false;
		selected = free_outputs;
	} else {
		return;
	}

	int i = rand() % selected.size();
	/*if (selected[i].id != 8){ // TODO: tray-8 --> which is the input-tray#1 is reserved for the 3D printer and conveyor belt scenario! See newPackageInputOnConveyor service below
		tray = getTray(selected[i].id);
	}else{
		return;
	}*/
	tray = getTray(selected[i].id);

	ROS_INFO("[package generator] Next package generation: %s!", input_chosen ? "Input" : "Output");
	bool success = false;

	auto_smart_factory::Package package;
	unsigned int index;
	if(doRequestPlans) {
		index = requestPlans.front().second;
	} else {
		//choose package_type randomly
		if(tray.package_type == 0) {
			index = rand() % packageConfigs.size();
		} else {
			index = tray.package_type - 1;
		}
	}
	package.type_id = packageConfigs[index].id;

	if(input_chosen) {
		package.id = externalPackages[index].front();
		success = newPackageInput(tray, package);
		if(success) {
			externalPackages[index].pop_front();
			if(doRequestPlans) {
				requestPlans.pop_front();
			}
		}
	} else {
		success = newPackageOutput(tray.id, package);
	}

	if(success) {
		updateTimer();
	}
}

void PackageGenerator::initializePackages() {
	externalPackages.clear();
	for(int i = 0; i < packageConfigs.size(); i++) {
		std::deque<int> deque;
		for(int j = 0; j < warehouseConfig.trays.size(); j++)
			deque.push_back(j + 1);
		externalPackages.push_back(deque);
	}
}

void PackageGenerator::initializeRequestPlan() {
	for(int i = 0; i < 22; i++)
		requestPlans.push_back(RequestPlan("input", 0));
}

bool PackageGenerator::getStorageInformation() {
	std::string srv_name = "storage_management/get_storage_information";
	ros::ServiceClient client = n.serviceClient<
			auto_smart_factory::GetStorageState>(srv_name.c_str());
	auto_smart_factory::GetStorageState srv;
	ros::service::waitForService(srv_name.c_str());
	if(client.call(srv)) {
		if(!hasStorageState
		   || storageState.stamp < srv.response.state.stamp) {
			storageState = srv.response.state;
			hasStorageState = true;
			ROS_INFO("[package generator] Storage information succesfully received & updated!");
			return true;
		} else {
			ROS_WARN("[package generator] Received storage information is outdated!");
			return false;
		}
	} else {
		ROS_ERROR("[package generator] Failed to call service %s!", srv_name.c_str());
		return false;
	}
}

void PackageGenerator::updateTrayState(
		const auto_smart_factory::StorageUpdate& msg) {
	if(msg.action == auto_smart_factory::StorageUpdate::DERESERVATION &&
	   msg.state.occupied && getTray(msg.state.id).type == "output") {
		clearOutput(msg.state);
	}

	for(int i = 0; i < storageState.tray_states.size(); i++)
		if(storageState.tray_states[i].id == msg.state.id) {
			storageState.tray_states[i].occupied = msg.state.occupied;
			storageState.tray_states[i].available = msg.state.available;
			if(msg.state.package.type_id != 0 && msg.state.package.id != 0) {
				storageState.tray_states[i].package = msg.state.package;
			}
			ROS_DEBUG("[package generator] State of tray[%i] updated: action: %u - pkg%d_%d (pkg%d_%d)!",
			          msg.state.id, msg.action, msg.state.package.type_id,
			          msg.state.package.id, storageState.tray_states[i].package.type_id,
			          storageState.tray_states[i].package.id);
			return;
		}
	ROS_ERROR("[package generator] Tray[%i] does not exist!", msg.state.id);
}

bool PackageGenerator::isTimeForGeneration() {
	timeval time;
	gettimeofday(&time, 0);
	if(lastTimestamp == 0 || time.tv_sec - lastTimestamp >= breakDuration) {
		return true;
	}
	return false;
}

void PackageGenerator::updateTimer() {
	timeval time;
	gettimeofday(&time, 0);
	lastTimestamp = time.tv_sec;
}


std::vector<auto_smart_factory::TrayState> PackageGenerator::getFreeStorages(
		std::string tray_type) {
	std::vector<auto_smart_factory::TrayState> selected_trays;
	for(int i = 0; i < storageState.tray_states.size(); i++) {
		if(!storageState.tray_states[i].occupied && storageState.tray_states[i].available) {
			if(tray_type != "charging station" &&
			   (tray_type == "all" ||
			    getTray(storageState.tray_states[i].id).type == tray_type)) {
				    selected_trays.push_back(storageState.tray_states[i]);
			}
		}
	}
	return selected_trays;
}

// This function is to input the package directly onto the conveyor (the 1st conveyor only). It pops up from the printer side
// as if it is generated by the printer. Currently only the first conveyor and 1st input tray only receives the inputs from conveyor.
// The rest of pkgs directly pops up on the input tray for the tray#2, tray#3 and tray#4. 
bool PackageGenerator::newPackageInputOnConveyor(auto_smart_factory::Tray tray, auto_smart_factory::Package package) {

	TrayAllocatorPtr allocatedInputTray = std::make_shared<TrayAllocator>(tray.id);

	// check if input tray reservation was successful
	if(!allocatedInputTray->isValid()) {
		ROS_DEBUG("[package generator] Allocation of input tray %d was not successful!", tray.id);
		return false;
	}

	ROS_INFO("[package generator] Successfully allocated input tray %d.", tray.id);

	// check if allocated input tray is still free (not occupied)
	auto_smart_factory::GetTrayState srv;
	srv.request.trayId = tray.id;
	if(!trayStateClient.call(srv) || srv.response.state.occupied) {
		ROS_WARN("[package generator] Checking allocated input tray failed. Occupied = %d",
		         srv.response.state.occupied);
		return false;
	}

	// set package
	if(!allocatedInputTray->setPackage(package)) {
		ROS_WARN("[package generator] Setting package failed. Package type and ID was:%d_%d", package.type_id,
		         package.id);
		return false;
	} else {
		ROS_INFO("[package generator] Package ID with: %d and TYPE: %d was set to TRAY_ID:%d", package.id, package.type_id, tray.id);
	}

	// move package
	// Drop height: Tray height (input tray) = 0.3 m + Half package height = 0.25 / 2 = 0.125 m
	std::cout << "tray IDs that the pkgs are inputted:  " << tray.id << std::endl;
	if(!movePackage(4 - 0.15, 0.80, 1,
	                package)) { //TODO: currently the locations are hard coded to move the pkg on conveyor which runs and makes it fall to input tray-01 !! this will be fixed according to the trays !
		//if (!movePackage(tray.x + 0.50 , tray.y - 4.85, 1 , package)) {
		ROS_WARN("[package generator] Moving package failed.");
		return false;
	}

	// wait for package to actually be moved
	auto_smart_factory::TrayState inputTrayState = getTrayState(tray.id);
	while(!inputTrayState.occupied ||
	      inputTrayState.package.type_id != package.type_id ||
	      inputTrayState.package.id != package.id) {
		if(!inputTrayState.occupied)
			ROS_WARN("[package generator] Input tray %d is still not occupied", tray.id);
		else
			ROS_WARN("[package generator] Input tray %d has the wrong package."
			         "Requested package type: %d and ID: %d. Received package type:%d, and ID: %d",
			         tray.id, package.type_id, package.id, inputTrayState.package.type_id, inputTrayState.package.id);
		ros::Duration(0.5).sleep();
		inputTrayState = getTrayState(tray.id);
	}

	// keep allocated tray alive until here
	allocatedInputTray = nullptr;

	// tell task planner
	std::string srv_name = "task_planner/new_input_task";
	ros::ServiceClient client = n.serviceClient<
			auto_smart_factory::NewPackageInput>(srv_name.c_str());
	auto_smart_factory::NewPackageInput packageInputSrv;
	packageInputSrv.request.input_tray_id = tray.id;
	packageInputSrv.request.package = package;

	while(!client.call(packageInputSrv)) {
		ROS_ERROR("[package generator] Failed to call service %s!", srv_name.c_str());
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("[package generator] Input request created at tray %d.", tray.id);

	return true;
}

bool PackageGenerator::newPackageInput(auto_smart_factory::Tray tray, auto_smart_factory::Package package) {

	TrayAllocatorPtr allocatedInputTray = std::make_shared<TrayAllocator>(tray.id);

	// check if input tray reservation was successful
	if(!allocatedInputTray->isValid()) {
		ROS_DEBUG("[package generator] Allocation of input tray %d was not successful!", tray.id);
		return false;
	}

	ROS_INFO("[package generator] Successfully allocated input tray %d.", tray.id);

	// check if allocated input tray is still free (not occupied)
	auto_smart_factory::GetTrayState srv;
	srv.request.trayId = tray.id;
	if(!trayStateClient.call(srv) || srv.response.state.occupied) {
		ROS_WARN("[package generator] Checking allocated input tray failed. Occupied = %d",
		         srv.response.state.occupied);
		return false;
	}

	// set package
	if(!allocatedInputTray->setPackage(package)) {
		ROS_WARN("[package generator] Setting package failed. Package type and ID was:%d_%d", package.type_id,
		         package.id);
		return false;
	} else {
		ROS_INFO("[package generator] Package ID with: %d and TYPE: %d was set to TRAY_ID:%d", package.id, package.type_id, tray.id);
	}

	// move package
	// Drop height: Tray height (input tray) = 0.3 m + Half package height = 0.25 / 2 = 0.125 m
	std::cout << "tray IDs that the pkgs are inputted:  " << tray.id << std::endl;
	if(!movePackage(tray.x, tray.y, 0.4, package)) {
		ROS_WARN("[package generator] Moving package failed.");
		return false;
	}

	// wait for package to actually be moved
	auto_smart_factory::TrayState inputTrayState = getTrayState(tray.id);
	while(!inputTrayState.occupied ||
	      inputTrayState.package.type_id != package.type_id ||
	      inputTrayState.package.id != package.id) {
		if(!inputTrayState.occupied)
			ROS_WARN("[package generator] Input tray %d is still not occupied", tray.id);
		else
			ROS_WARN("[package generator] Input tray %d has the wrong package."
			         "Requested package type: %d and ID: %d. Received package type:%d, and ID: %d",
			         tray.id, package.type_id, package.id, inputTrayState.package.type_id, inputTrayState.package.id);
		ros::Duration(0.5).sleep();
		inputTrayState = getTrayState(tray.id);
	}

	// keep allocated tray alive until here
	allocatedInputTray = nullptr;

	// tell task planner
	std::string srv_name = "task_planner/new_input_task";
	ros::ServiceClient client = n.serviceClient<
			auto_smart_factory::NewPackageInput>(srv_name.c_str());
	auto_smart_factory::NewPackageInput packageInputSrv;
	packageInputSrv.request.input_tray_id = tray.id;
	packageInputSrv.request.package = package;

	while(!client.call(packageInputSrv)) {
		ROS_ERROR("[package generator] Failed to call service %s!", srv_name.c_str());
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("[package generator] Input request created at tray %d.", tray.id);

	return true;
}

bool PackageGenerator::newPackageOutput(int output_tray_id,
                                        auto_smart_factory::Package package) {
	std::string srv_name = "task_planner/new_output_task";
	ros::ServiceClient client = n.serviceClient<
			auto_smart_factory::NewPackageOutput>(srv_name.c_str());
	auto_smart_factory::NewPackageOutput srv;
	srv.request.output_tray_id = output_tray_id;
	srv.request.package = package;

	if(client.call(srv)) {
		if(srv.response.success) {
			ROS_INFO("[package generator] New output request generated at tray %i!", output_tray_id);
			return true;
		} else {
			ROS_INFO("[package generator] Output tray %i is not available for an output request!", output_tray_id);
			return false;
		}
	} else {
		ROS_ERROR("[package generator] Failed to call service %s!", srv_name.c_str());
		return false;
	}
}

void PackageGenerator::clearOutput(auto_smart_factory::TrayState tray_state) {
	ROS_INFO("[package generator] Clear tray_id: %d, pkg%d_%d", tray_state.id, tray_state.package.type_id, tray_state.package.id);
	// reserve output tray
	TrayAllocator allocatedOutputTray(tray_state.id);

	if(!allocatedOutputTray.isValid()) {
		ROS_WARN("[package generator] Occupied output tray %d could not be allocated in order to remove package!",
		         tray_state.id);
		return;
	}

	// sleep 1 sec to make it look nicer
	ros::Duration(1.0).sleep();

	//move packages to fix position
	if(movePackage(0.0, 0.0, -1.0, tray_state.package)) {
		ROS_INFO("[package generator] Output %i cleared successfully!", tray_state.id);

		// wait for package to actually be moved
		auto_smart_factory::TrayState outputTrayState = tray_state;
		while(outputTrayState.occupied) {
			ROS_INFO("[package generator] Output tray %d is still occupied. Wait for the package to be moved...", tray_state.id);
			ros::Duration(0.5).sleep();
			outputTrayState = getTrayState(tray_state.id);
		}

		// clear package information
		allocatedOutputTray.setPackage(auto_smart_factory::Package());

		putBackPackage(tray_state.package);
	} else
		ROS_INFO("[package generator] Failed to clear Output %i!", tray_state.id);
}

bool PackageGenerator::movePackage(float x, float y, float z, auto_smart_factory::Package package) {
	std::string srv_name = "package_manipulator/move_package";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::MovePackage>(srv_name.c_str());
	auto_smart_factory::MovePackage srv;
	std::string id = "pkg" + std::to_string(package.type_id) + "_"
	                 + std::to_string(package.id);
	srv.request.package_id = id;
	srv.request.x = x;
	srv.request.y = y;
	srv.request.z = z;
	ros::service::waitForService(srv_name.c_str());
	if(client.call(srv)) {
		if(srv.response.success) {
			ROS_INFO("[package generator] %s has been moved successfully!", id.c_str());
			return true;
		} else
			ROS_WARN("[package generator] %s has not been moved!", id.c_str());
	} else
		ROS_ERROR("[package generator] Failed to call service %s %s",
		          srv_name.c_str(), id.c_str());
	return false;
}

void PackageGenerator::putBackPackage(auto_smart_factory::Package package) {
	for(int i = 0; i < packageConfigs.size(); i++)
		if(packageConfigs[i].id == package.type_id) {
			externalPackages[i].push_back(package.id);
			return;
		}
	std::string id = "pkg" + std::to_string(package.type_id) + "_"
	                 + std::to_string(package.id);
	ROS_ERROR("[package generator] Failed to put back package %s!", id.c_str());
}

auto_smart_factory::Tray PackageGenerator::getTray(unsigned int id) {
	for(int i = 0; i < warehouseConfig.trays.size(); i++) {
		if(warehouseConfig.trays[i].id == id) {
			return warehouseConfig.trays[i];
		}
	}
	ROS_ERROR("[package generator] There is no tray with id=%i!", id);
	auto_smart_factory::Tray tray;
	return tray;
}

auto_smart_factory::TrayState PackageGenerator::getTrayState(unsigned int tray_id) {
	std::string srv_name = "/storage_management/get_tray_state";
	ros::ServiceClient client = n.serviceClient<auto_smart_factory::GetTrayState>(srv_name.c_str());
	auto_smart_factory::GetTrayState srv;
	srv.request.trayId = tray_id;
	ros::service::waitForService(srv_name.c_str());
	if(client.call(srv)) {
		ROS_DEBUG("[package generator] State of tray %u has been received successfully!",
		          tray_id);
		return srv.response.state;
	} else {
		ROS_ERROR("[package generator] Failed to call service %s!", srv_name.c_str());
		auto_smart_factory::TrayState s;
		return s;
	}
}
