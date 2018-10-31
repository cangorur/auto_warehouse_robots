#include <config_server/PackageConfigServer.h>
#include <boost/property_tree/json_parser.hpp>

using namespace boost::property_tree;

PackageConfigServer::PackageConfigServer() {
	ros::NodeHandle nh("~");
	std::string packageConfigFileName;

	if(!nh.getParam("package_config_file", packageConfigFileName)) {
		ROS_FATAL("No package configuration file name given!");
		ros::shutdown();
		return;
	}

	readPackageConfigs(packageConfigFileName);

	configService = nh.advertiseService("get_package_configurations", &PackageConfigServer::configCallback, this);
}

PackageConfigServer::~PackageConfigServer() {
}

bool PackageConfigServer::configCallback(auto_smart_factory::GetPackageConfigurations::Request& req, auto_smart_factory::GetPackageConfigurations::Response& res) {
	res.configs = packageConfigs;
	return true;
}

void PackageConfigServer::readPackageConfigs(std::string file) {
	ptree configTree;

	try {
		read_json(file, configTree);
	} catch(json_parser::json_parser_error &e) {
		ROS_FATAL("Cannot read package configuration file %s. Message: %s", file.c_str(), e.what());
		return;
	}

	packageConfigs.clear();

	for(const auto& packageType : configTree) {
		auto_smart_factory::PackageConfiguration packageConfig;
		packageConfig.id = std::stoi(packageType.first);
		packageConfig.width = packageType.second.get<float>("width");
		packageConfig.height = packageType.second.get<float>("height");
		packageConfig.weight = packageType.second.get<float>("weight");

		packageConfigs.push_back(packageConfig);
	}
}
