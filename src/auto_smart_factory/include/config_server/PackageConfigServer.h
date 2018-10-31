#ifndef AUTO_SMART_FACTORY_SRC_PACKAGECONFIGSERVER_H_
#define AUTO_SMART_FACTORY_SRC_PACKAGECONFIGSERVER_H_

#include <ros/ros.h>
#include <auto_smart_factory/PackageConfiguration.h>
#include <auto_smart_factory/GetPackageConfigurations.h>

/**
 * This class reads the package configuration file and provides a service to deliver the package
 * configurations to other components.
 */
class PackageConfigServer {
public:
	PackageConfigServer();
	virtual ~PackageConfigServer();

protected:
	/**
	 * Package configuration service callback function.
	 * @param req Request object
	 * @param res Response object
	 * @return Success (always true)
	 */
	bool configCallback(auto_smart_factory::GetPackageConfigurations::Request& req, auto_smart_factory::GetPackageConfigurations::Response& res);

	/**
	 * Reads the JSON formatted package configurations from file.
	 * The configurations are stored internally.
	 * @param file Path to configuration file
	 */
	void readPackageConfigs(std::string file);

	/**
	 * Package configurations
	 */
	std::vector<auto_smart_factory::PackageConfiguration> packageConfigs;

	/**
	 * Package configurations retrieval server
	 */
	ros::ServiceServer configService;
};

#endif /* AUTO_SMART_FACTORY_SRC_PACKAGECONFIGSERVER_H_ */
