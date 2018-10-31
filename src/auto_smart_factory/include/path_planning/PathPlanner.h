#ifndef PATHPLANNER_INCLUDED
#define PATHPLANNER_INCLUDED


#include "ros/ros.h"
#include "ros/topic.h"
#include <string>
#include <vector>
#include <map>
#include <list>
#include <exception>
#include <stdexcept>
#include <time.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <auto_smart_factory/PlannedPath.h>
#include <auto_smart_factory/InitPathPlanner.h>
#include <auto_smart_factory/RequestNewPath.h>
#include <auto_smart_factory/RoadmapGraph.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/shared_ptr.hpp>
#include <sys/time.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

using namespace boost;

using cost = double; // typedef for euclidean distance used in graph
// typedefs for generating graphs with boost
/*
 *  Adjacency list provides template parameters to select different configurations
 *  for graph.
 *             listS: represents the edge list for each of vertices.
 *             VecS:  represents the vertex list.
 *             undirectedS: selects graph type as undirected.
 *             no_property: this parameter can be used to specify internal property for vertex.
 *             property<edge_weight_t, cost>: Uses edge weight(euclidean distance) to select edge
 *  @todo Students are free to create their own graph given the roadmap graph generated. Please refer
 *  to the generate_graph function.
 */
typedef adjacency_list<listS, vecS, undirectedS, no_property,
                        property<edge_weight_t, cost> > mygraph_t;

/*
 * Property_map: used to select the properties associated with graph edges and vertices.
 * In our case, we have edge weight is associated with graph
 */
typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;

// vertex_descriptor refers to vertices in boost graph.
typedef mygraph_t::vertex_descriptor vertex;

// edge_descriptor refers to edges in boost graph
typedef mygraph_t::edge_descriptor edge_descriptor;


/*
 * Path Planner gives path chunks to the robot agents. It will be implemented with the strategies and planning
 * algorithms from the students.
 */
class PathPlanner{

public:
			
	PathPlanner();

    //virtual ~PathPlanner();

    /*
     * Generates graph using boost library from graph data received
     * from roadmap graph message
     * @todo Students are free to create their own graph solutions
    */
    void generate_graph();


private:
    //graph parameters
	int num_nodes;
	int num_edges;
	mygraph_t graph;
	std::vector<int> x_coords_int;
	std::vector<int> y_coords_int;
	std::vector<int> start_nodes;
	std::vector<int> end_nodes;
	std::vector<double> weight;
	double height;
	double width;
	struct Coordinates {
		double x, y;
	};

    //set to true if graph parameters are received from rooadmap graph msg
	bool is_ready = false;

	//Structure for coordinates in map // required to calculate heuristic h
	Coordinates *coordinates;

    // list  of robots
    std::vector<std::string> robot_id;

    //callback function for Initialize service
    //bool initialize(auto_smart_factory::InitPathPlannerRequest &req, auto_smart_factory::InitPathPlannerResponse &res);

    // call back function for roadmap graph message
    void initialize_roadmap_graph(const auto_smart_factory::RoadmapGraph &rmg);

		
    // Path planner publishes planned path to trafic monitor
	ros::Publisher plannedpathPub;
		
    // Subscriber to roadmap graph message
	ros::Subscriber roadmapSub;


    //Service for initializing the node
	//ros::ServiceServer initServer;

    //Service for new path
	ros::ServiceServer newpathServer;
		
    // ID of this agent
	std::string agentID; 

    /*
     * Service handler for a new path request. The request comes from an Agent (see Agent.cpp for the service call)
     * @todo This method will be implemented for your own pathplanning algorithm. Hint: You will use the information
     * provided such as the roadmap graph generated and agentID and other data agent sends following the necessary
     * request inputs as below. But it is suggested to double-check the Agent.cpp if they are correctly provided.
     *      Args: req:
     *            next_chunk: a boolean variable: High if path if requested for first time otherwise low
     *            agent_id: id of robot which is requesting for a path.
     *            start_node, end node : Starting and final position of robot.
     *      Returns:res:
     *            Path: chunk path: Path between first two consecutive random points: refer to
     *            report for  details.
     *            path_length: number of nodes in path
    */
	bool PathFinding(auto_smart_factory::RequestNewPath::Request &req,
		auto_smart_factory::RequestNewPath::Response &res);
};
#endif // PATHPLANNER_INCLUDED
