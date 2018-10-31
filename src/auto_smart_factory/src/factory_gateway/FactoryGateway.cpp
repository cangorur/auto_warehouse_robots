/*
 * TaskPlanner.cpp
 *
 *  Created on: 25.08.2017
 *      Author: Orhan Can Görür
 *      Email: orhan-can.goeruer@dai-labor.de
 */

#include <ros/ros.h>
// include <package_name/service_type_name.h>
#include <std_srvs/Trigger.h>

#include <stdlib.h> 				// for rand() and RAND_MAX
#include <string.h>
#include <iostream>

#include "simple_web_socket/server_ws.hpp"
#include "simple_web_socket/client_ws.hpp"

typedef SimpleWeb::SocketServer<SimpleWeb::WS> WsServer;
typedef SimpleWeb::SocketClient<SimpleWeb::WS> WsClient;

#include <factory_gateway/FactoryGateway.h>

#include <auto_smart_factory/Package.h>
#include <auto_smart_factory/Tray.h>


using namespace std;

FactoryGateway::FactoryGateway() {
	ros::NodeHandle pn("~");
	initialize();

	ROS_INFO("FactoryGateway is created...");
}

FactoryGateway::~FactoryGateway() {
}

void FactoryGateway::initialize() {
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	storageStates = n.serviceClient<auto_smart_factory::GetStorageState>("/storage_management/get_storage_information");
	freeChargingStations = n.serviceClient<auto_smart_factory::GetFreeChargingStations>("/charging_management/get_free_charging_stations");
	toggleConveyor1 = n.serviceClient<std_srvs::Trigger>("/conveyor_1/switch_on_off");
	toggleConveyor2 = n.serviceClient<std_srvs::Trigger>("/conveyor_2/switch_on_off");
	generateNewPackageClient = n.serviceClient<auto_smart_factory::NewPackageGenerator>(
						"package_generator/new_package_generator");
	// subscribe to the robot battery topics
	// TODO: new robot battery topic/service will be written under charging management and it will be initialized here called below necessary.
	robot1_battery = n.subscribe("/robot_1/battery", 1000, &FactoryGateway::receiveRobot1Battery, this);
	robot2_battery = n.subscribe("/robot_2/battery", 1000, &FactoryGateway::receiveRobot2Battery, this);
	robot3_battery = n.subscribe("/robot_3/battery", 1000, &FactoryGateway::receiveRobot3Battery, this);
	robot4_battery = n.subscribe("/robot_4/battery", 1000, &FactoryGateway::receiveRobot4Battery, this);
	robot5_battery = n.subscribe("/robot_5/battery", 1000, &FactoryGateway::receiveRobot5Battery, this);
	robot6_battery = n.subscribe("/robot_6/battery", 1000, &FactoryGateway::receiveRobot6Battery, this);
	robot7_battery = n.subscribe("/robot_7/battery", 1000, &FactoryGateway::receiveRobot7Battery, this);
	robot8_battery = n.subscribe("/robot_8/battery", 1000, &FactoryGateway::receiveRobot8Battery, this);
	
	/*
	registerAgentServer = pn.advertiseService("register_agent",
			&TaskPlanner::registerAgent, this);
	statusUpdatePub = pn.advertise<TaskPlannerState>("status", 1);
	*/

	ROS_INFO("Warehouse Gateway is initialized.");
}

/*
 * SETTING WEB SOCKET SERVER
 */
void FactoryGateway::SetWebSocketServer(){
	
	srand (time(NULL));
	//WebSocket (WS)-server at port 9090 using 1 thread
  
	WsServer server;
	server.config.port = 8887;
	
	auto& echo=server.endpoint["^/?$"];
	cout << "websocket server is being initialized" << endl;
	echo.on_open=[](shared_ptr<WsServer::Connection> connection) {
		cout << "Warehouse Gateway Server: Opened connection " << (size_t)connection.get() << endl;
	};

	echo.on_message=[&](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {
		
		ros::spinOnce(); // TODO: this is placed for robot battery states rostopic. It will be removed from here once a service for that is implemented.
		
		cout << "Warehouse Server: Message received !"<< endl;
		auto message_str=message->string();
		stringstream message_ss;
		message_ss << message_str;
		boost::property_tree::ptree message_pt; // json ptree object
		cout << "JSON object received:" << message_str << endl;
		
		try {
			boost::property_tree::read_json(message_ss, message_pt);
		} catch(boost::property_tree::json_parser::json_parser_error &e) {
			ROS_FATAL("Cannot parse the message to Json. Error: %s", e.what());
			return;
		}
		
		// Calling the right function (rosservice) to respond to the request
		string sender_name = message_pt.get<string>("requestee.id");
		cout << "received message from:" << sender_name << endl;
		if (sender_name == "storage-service"){
			message_pt = GetStateOfStorages(message_pt);
		}
		if (sender_name == "input-container-service"){
			message_pt = GetStateOfInputContainers(message_pt);
		}
		if (sender_name == "delivery-container-service"){
			message_pt = GetStateOfDeliveryContainers(message_pt);
		}
		if (sender_name == "chargingstation-service"){
			message_pt = GetStateOfChargingStations(message_pt);
		}
		if (sender_name == "deliveryrobot-service"){
			message_pt = GetStateOfDeliveryRobots(message_pt);
		}
		if (sender_name == "conveyorbelt-service" || sender_name == "conveyerbelt-service"){
			message_pt = ToggleConveyorBelts(message_pt);
		}
		
		// Translating the responded json object into string to send via websocket
		auto send_stream = make_shared<WsServer::SendStream>();
		ostringstream buf_stream;
		boost::property_tree::write_json(buf_stream, message_pt, false);
		*send_stream << buf_stream.str();
		server.send(connection, send_stream, [](const SimpleWeb::error_code& ec){
			if(ec) {
				cout << "Server: Error sending message. " <<
					"Error: " << ec << ", error message: " << ec.message() << endl;
			}
		});

	};
	
	//See RFC 6455 7.4.1. for status codes
	echo.on_close=[](shared_ptr<WsServer::Connection> connection, int status, const string& /*reason*/) {
		cout << "Server: Closed connection " << (size_t)connection.get() << " with status code " << status << endl;
	};
	
	//See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
	echo.on_error=[](shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code& ec) {
		cout << "Server: Error in connection " << (size_t)connection.get() << ". " << 
				"Error: " << ec << ", error message: " << ec.message() << endl;
	};
	
	server.start();
	
}
boost::property_tree::ptree FactoryGateway::ToggleConveyorBelts(boost::property_tree::ptree message_pt){
	
	boost::property_tree::ptree conveyor_node;
	boost::property_tree::ptree conveyor_status;
	std_srvs::Trigger::Request req1;
	std_srvs::Trigger::Response res1;
	
	string sender_name = message_pt.get<string>("requester.id");
	
	//TODO: implement switch on / off request. It is already in the .json request format.
	if(sender_name == "conveyorbelt-driver-1" || sender_name == "conveyerbelt-driver-1"){
		
		//cout << "received message!:  " << message_pt.get<string>("request.switch") << "  ||| Current conveyor state" << conveyor1_state << endl;
		if ((message_pt.get<string>("request.switch") != "") && 
				!(message_pt.get<string>("request.switch") == conveyor1_state)){
			toggleConveyor1.call(req1, res1);
			if (res1.success == true){
				conveyor1_state = "on";
				conveyor1_speed = "0.1";
				conveyor_status.put("status", conveyor1_state);
				conveyor_status.put("speed", conveyor1_speed);
				// generating a package for the conveyor ON request !
				//TODO: currently fixed type of packages to the fixed tray are generated
				auto_smart_factory::Package package;
				auto_smart_factory::Tray tray;
				
				auto_smart_factory::NewPackageGenerator srv;
				// int index = rand() % 2;
				// currently only a fixed type is requested
				package.type_id = 2;
				tray.id = 8; //always send the pkgs to tray-1
				tray.type = "input";
				
				srv.request.package = package;
				srv.request.tray = tray;
				generateNewPackageClient.call(srv.request, srv.response);
				std::cout << "PACKAGE REQUEST SUCCESS:?" << srv.response << std::endl;
			}else{
				conveyor1_state = "off";
				conveyor1_speed = "0.0";
				conveyor_status.put("status", conveyor1_state);
				conveyor_status.put("speed", conveyor1_speed);
			}
		}
		else{
			conveyor_status.put("status", conveyor1_state);
			conveyor_status.put("speed", conveyor1_speed);
		}
		conveyor_node.push_back(make_pair("conveyorbelt-1", conveyor_status));
		message_pt.put_child("response", conveyor_node);
	}
	if(sender_name == "conveyorbelt-driver-2" || sender_name == "conveyerbelt-driver-2"){
		if ((message_pt.get<string>("request.switch") != "") && 
						!(message_pt.get<string>("request.switch") == conveyor2_state)){
			toggleConveyor2.call(req1, res1);
			if (res1.success == true){
				conveyor2_state = "on";
				conveyor2_speed = "0.1";
				conveyor_status.put("status", conveyor2_state);
				conveyor_status.put("speed", conveyor2_speed);
			}else{
				conveyor2_state = "off";
				conveyor2_speed = "0.0";
				conveyor_status.put("status", conveyor2_state);
				conveyor_status.put("speed", conveyor2_speed);
			}
		}
		else{
			conveyor_status.put("status", conveyor2_state);
			conveyor_status.put("speed", conveyor2_speed);
		}
		conveyor_node.push_back(make_pair("conveyorbelt-2", conveyor_status));
		message_pt.put_child("response", conveyor_node);
	}
	
	return message_pt;
}
boost::property_tree::ptree FactoryGateway::GetStateOfStorages(boost::property_tree::ptree message_pt){
	//TODO: only send the storage trays
	/*
	 * Calling out the get storage information ros service
	 */
	auto_smart_factory::GetStorageState srv_storage;
	storageStates.call(srv_storage);
	vector<auto_smart_factory::TrayState> currentTrayStates = srv_storage.response.state.tray_states;
	
	boost::property_tree::ptree tray_node;
	for (int i=0; i < currentTrayStates.size(); i++){
		if (currentTrayStates[i].id > 11 && currentTrayStates[i].id < 31) {//from number 8 to number 16 the trays are container input trays
			string tray_id = "tray-" + to_string(currentTrayStates[i].id - 11);
			bool status = currentTrayStates[i].occupied;
			string IsOccupied;
			if (status == true){
				IsOccupied = "Occupied";
			} else {
				IsOccupied = "Free";
			}

			boost::property_tree::ptree tray_status;
			tray_status.put("status", IsOccupied);
			tray_node.push_back(make_pair(tray_id, tray_status));
		}
	}
	boost::property_tree::ptree storage_state_node;
	storage_state_node.push_back(make_pair("storagestates", tray_node));
	message_pt.put_child("response", storage_state_node);
	ROS_INFO("The up-to-date Storage States have been shared!");
	return message_pt;
}


boost::property_tree::ptree FactoryGateway::GetStateOfInputContainers(boost::property_tree::ptree message_pt){

	auto_smart_factory::GetStorageState srv_storage;
	storageStates.call(srv_storage);
	vector<auto_smart_factory::TrayState> currentTrayStates = srv_storage.response.state.tray_states;
	
	boost::property_tree::ptree tray_node;
	for (int i=0; i < currentTrayStates.size(); i++){
		if (currentTrayStates[i].id < 12) {//from number 8 to number 16 the trays are container input trays
			string tray_id = "tray-" + to_string(currentTrayStates[i].id - 7);
			bool status = currentTrayStates[i].occupied;
			string IsOccupied;
			if (status == true){
				IsOccupied = "Occupied";
			} else {
				IsOccupied = "Free";
			}

			boost::property_tree::ptree tray_status;
			tray_status.put("status", IsOccupied);
			tray_node.push_back(make_pair(tray_id, tray_status));
		}
	}
	// TODO: constant four trays are added as Free: change this when new trays are added!
	for (int i = 0; i < 4; i++){
		boost::property_tree::ptree tray_status;
		tray_status.put("status", "Free");
		tray_node.push_back(make_pair("tray-" + to_string(i+5), tray_status));
	}	
	
	boost::property_tree::ptree input_state_node;
	input_state_node.push_back(make_pair("inputcontainerstates", tray_node));
	message_pt.put_child("response", input_state_node);
	ROS_INFO("The up-to-date Container States have been shared!");
	return message_pt;
}


boost::property_tree::ptree FactoryGateway::GetStateOfDeliveryContainers(boost::property_tree::ptree message_pt){

	auto_smart_factory::GetStorageState srv_storage;
	storageStates.call(srv_storage);
	vector<auto_smart_factory::TrayState> currentTrayStates = srv_storage.response.state.tray_states;
	
	boost::property_tree::ptree tray_node;
	for (int i=0; i < currentTrayStates.size(); i++){
		if (currentTrayStates[i].id > 30) {//from number 8 to number 16 the trays are container input trays
			string tray_id = "tray-" + to_string(currentTrayStates[i].id - 30);
			bool status = currentTrayStates[i].occupied;
			string IsOccupied;
			if (status == true){
				IsOccupied = "Occupied";
			} else {
				IsOccupied = "Free";
			}

			boost::property_tree::ptree tray_status;
			tray_status.put("status", IsOccupied);
			tray_node.push_back(make_pair(tray_id, tray_status));
		}
	}
	
	boost::property_tree::ptree delivery_state_node;
	delivery_state_node.push_back(make_pair("deliverycontainerstates", tray_node));
	message_pt.put_child("response", delivery_state_node);
	ROS_INFO("The up-to-date Container States have been shared!");
	return message_pt;
}

boost::property_tree::ptree FactoryGateway::GetStateOfChargingStations(boost::property_tree::ptree message_pt){
	
	//TODO: this function calls the services and subscriptions to hold the last up-to-date values
	//BASED ON THE MESSAGE RECEIVED ???
	/*
	auto_smart_factory::GetTrayState srv;
	srv.request.trayId = tray.id;
	if (!trayStateClient.call(srv) || srv.response.state.occupied) {
		ROS_WARN("[package generator] Checking allocated input tray failed. Occupied = %d", 
				srv.response.state.occupied);
		return false;
	}*/
	auto_smart_factory::GetFreeChargingStations srv_charging;
	freeChargingStations.call(srv_charging);
	boost::property_tree::ptree station_node;
	
	vector<int> occupiedIds_array;
	for (int i=0; i < srv_charging.response.charging_stations.size(); i++){
		int station_id = srv_charging.response.charging_stations[i].id;
		occupiedIds_array.push_back(station_id);
		
		string station_id_str = "chargingstation-" + to_string(station_id + 1);
		
		boost::property_tree::ptree chargingstation_status;
		chargingstation_status.put("status", "Free");
		station_node.push_back(make_pair(station_id_str, chargingstation_status));
	}
	sort(occupiedIds_array.begin(), occupiedIds_array.end()); //TODO: after the get charging stations service will be updated this wont be needed anymore! issue#8
	for (int i=0; i < 8; i++){
		if (!(find(occupiedIds_array.begin(), occupiedIds_array.end(), i) != occupiedIds_array.end())){
			
			string station_id_str = "chargingstation-" + to_string(i + 1);
					
			boost::property_tree::ptree chargingstation_status;
			chargingstation_status.put("status", "Reserved");
			station_node.push_back(make_pair(station_id_str, chargingstation_status));
		}
	}
	
	boost::property_tree::ptree chargingstation_state_node;
	chargingstation_state_node.push_back(make_pair("chargingstationstates", station_node));
	message_pt.put_child("response", chargingstation_state_node);
	ROS_INFO("The up-to-date Charging Station occupancies has been shared!");
	return message_pt;
}

boost::property_tree::ptree FactoryGateway::GetStateOfDeliveryRobots(boost::property_tree::ptree message_pt){
	
	string sender_name = message_pt.get<string>("requester.id");
	boost::property_tree::ptree robot_status;
	boost::property_tree::ptree robot_node;

	if(sender_name == "deliveryrobot-driver-1"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[0]));
		robot_node.push_back(make_pair("deliveryrobot-1", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-2"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[1]));
		robot_node.push_back(make_pair("deliveryrobot-2", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-3"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[2]));
		robot_node.push_back(make_pair("deliveryrobot-3", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-4"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[3]));
		robot_node.push_back(make_pair("deliveryrobot-4", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-5"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[4]));
		robot_node.push_back(make_pair("deliveryrobot-5", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-6"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[5]));
		robot_node.push_back(make_pair("deliveryrobot-6", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-7"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[6]));
		robot_node.push_back(make_pair("deliveryrobot-7", robot_status));
		message_pt.put_child("response", robot_node);
	}
	if(sender_name == "deliveryrobot-driver-8"){
		robot_status.put("batteryLevel", to_string(robotBatteries_arr[7]));
		robot_node.push_back(make_pair("deliveryrobot-8", robot_status));
		message_pt.put_child("response", robot_node);
	}
	
	//robot_node.push_back(make_pair(tray_id, tray_status));
	
	ROS_INFO("The up-to-date requested delivery robot states have been shared!");
	return message_pt;
}

//==========================================================

void FactoryGateway::webSocketClient(string message) {

	WsClient client("localhost:8080");
	
	client.on_open=[&client]() {
	  string message="sending a message";
	  //*out_ << "Client: Opened connection" << endl;
	  cout << ">>> Warehouse Gateway Client: Sending message: \"" << message << "\"" << endl;
	
	  auto send_stream=make_shared<WsClient::SendStream>();
	  *send_stream << message;
	  client.send(send_stream);
	}; 
	
	client.on_message=[&client](shared_ptr<WsClient::Message> message) {
	  auto message_str=message->string();
	  cout << "Client: Message received: \"" << message_str << "\"" << endl;
	  //cout << "Client: Sending close connection" << endl;
	  client.send_close(1000);
	};
	
	client.on_close=[](int status, const string& /*reason*/) {
	  cout << "Client: Closed connection with status code " << status << endl;
	};
	  
	//See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
	client.on_error=[](const SimpleWeb::error_code& ec) {
	  cout << "Client: Error: " << ec << ", error message: " << ec.message() << endl;
	};
	
	client.start();
}

//==========================================================

//================rostopic callbacks========================
void FactoryGateway::receiveRobot1Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[0] =  msg.data;
}
void FactoryGateway::receiveRobot2Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[1] =  msg.data;
}
void FactoryGateway::receiveRobot3Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[2] =  msg.data;
}
void FactoryGateway::receiveRobot4Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[3] =  msg.data;
}
void FactoryGateway::receiveRobot5Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[4] =  msg.data;
}
void FactoryGateway::receiveRobot6Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[5] =  msg.data;
}
void FactoryGateway::receiveRobot7Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[6] =  msg.data;
}
void FactoryGateway::receiveRobot8Battery(const std_msgs::Float32& msg) {
	robotBatteries_arr[7] =  msg.data;
}

//==========================================================

