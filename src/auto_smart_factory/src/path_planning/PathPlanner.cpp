#include <path_planning/PathPlanner.h>

PathPlanner::PathPlanner(){
    ros::NodeHandle n;
    ROS_INFO("Path planner created...");

    //Receives road map data (see the callback function)
    roadmapSub = n.subscribe("roadmap_graph", 10,
                             &PathPlanner::initialize_roadmap_graph, this);

    ros::Rate r = ros::Rate(10);
    while(!is_ready) {
        ros::spinOnce();
        r.sleep();
    }

    generate_graph();

    // service publishes new path when requested (Agent request it) or publisher to publish the path generated
    this->newpathServer = n.advertiseService("path_planner/request_new_path", &PathPlanner::PathFinding, this);
    plannedpathPub =n.advertise<auto_smart_factory::PlannedPath>("path_planner/path", 10);

    ROS_INFO("Path planner initialized.");
}


void PathPlanner::initialize_roadmap_graph(const auto_smart_factory::RoadmapGraph &rmg) {

    is_ready = false;
    num_edges = rmg.num_edges;
    num_nodes = rmg.num_nodes;
    x_coords_int = rmg.x_coord_int;
    y_coords_int = rmg.y_coord_int;
    start_nodes = rmg.start_nodes;
    end_nodes = rmg.end_nodes;
    weight = rmg.lengths;
    height =rmg.height;
    width = rmg.width;
    coordinates = new Coordinates[num_nodes];
    int i = 0;
        for (auto it = rmg.coordinates.begin(); it != rmg.coordinates.end(); ++it) {
            coordinates[i].x = it->x;
            coordinates[i].y = it->y;
            i++;
        }
    is_ready = true;
    //ROS_WARN("Graph parameters initialized in path planner.");
}

void  PathPlanner::generate_graph() {


    //create edges pairs for graph
    typedef std::pair<int, int> edge;
    edge edge_ary[num_edges];
        for (int i = 0; i < num_edges; i++) {
            edge_ary[i] = edge(start_nodes.at(i), end_nodes.at(i));
        }

    mygraph_t gh(num_nodes);

    WeightMap weightmaph = get(edge_weight, gh);
        for (std::size_t j = 0; j < num_edges; ++j) {
            edge_descriptor e;
            bool inserted;
            boost::tie(e, inserted) = add_edge(edge_ary[j].first,
                                               edge_ary[j].second, gh);
            weightmaph[e] = weight[j];
        }

    // store copy of graph  be used in shortest path function;
    graph = gh;

    //ROS_WARN("Graph initialized in path planner.");

}

bool PathPlanner::PathFinding(auto_smart_factory::RequestNewPath::Request &req,
    auto_smart_factory::RequestNewPath::Response &res)
{

    ROS_INFO("in PathPlanner::PathFinding handles chunk request");

    // TODO: Here will be implemented for a proper path finding algorithm

    return true;
}


