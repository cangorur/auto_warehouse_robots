#ifndef AUTO_SMART_FACTORY_SRC_GRIPPER_H_
#define AUTO_SMART_FACTORY_SRC_GRIPPER_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include "std_srvs/Trigger.h"
#include "auto_smart_factory/GripperState.h"
#include "auto_smart_factory/Package.h"
#include "auto_smart_factory/MoveGripper.h"

class Agent;

/**
 * The gripper component calls a service of the MORSE Gripper to load or unload.
 * It magically just grips or releases packages in front of the gripper without handling 
 * any armature gestures.
 */
class Gripper{
public:
	/**
	 * Constructor that hands over the agents id and the publisher for the gripper state topic.
	 * @param _agent: agent this gripper instance belongs to
	 * @param gripper_state_pub: publisher for the gripper state topic
	 */
	Gripper(Agent* _agent, ros::Publisher *gripper_state_pub);
	virtual ~Gripper();

	/**
	 * Calls the gripper service to load or unload a package. 
	 * @param load: if package should load or unload
	 * @return True if the service has been called succesfully
	 */
	bool loadPackage(bool load);

protected:
	/**
	 * Splits a string at the given separator into a list of substrings.
	 * Helps to get to know the actual name of the package that is grasped 
	 * by splitting the MORSE message.
	 * @param text: the string to split
	 * @param sep: the char where the string should be splitted
	 * @return list of substrings
	 */
	std::vector<std::string> split(std::string &text, char sep);

	/// ROS Nodehandle
	ros::NodeHandle n;

	/// Publisher for the gripper state topic
	ros::Publisher *gripperStatePub;

    /// The agent to which this gripper is attached to
    Agent *agent;

    /// ID of the agent the gripper belongs to
	std::string agentID; 

	/// Actual gripped or attatched package - saved here till detatching
	auto_smart_factory::Package package;

	bool moveGripper(float x, float y, float z, auto_smart_factory::Package package);
};

#endif /* AUTO_SMART_FACTORY_SRC_GRIPPER_H_ */
