/**
 * This D star lite implementation is intended to replace the A* in path planner, therefore data structure
 * from boost library is used. 
 * The algorithm is based on [S. Koenig, 2002], code structure is currently referring to https://code.google.com/archive/p/dstarlite/
 */
#include <boost/graph/adjacency_list.hpp>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <ext/hash_map>

using namespace boost;
using namespace std;
using namespace __gnu_cxx;

// specify some typedefs for graph
using cost = double;
typedef adjacency_list<listS, vecS, undirectedS, no_property, property<edge_weight_t, cost> > mygraph_t;
typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
typedef mygraph_t::vertex_descriptor Vertex;
typedef mygraph_t::edge_descriptor Edge;

class state {
 public:
  Vertex vertex;
  int x;
  int y;
  pair<double,double> key;

  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }
  
  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }
  
  bool operator > (const state &s2) const {
    if (key.first-0.00001 > s2.key.first) return true;
    else if (key.first < s2.key.first-0.00001) return false;
    return key.second > s2.key.second;
  }

  bool operator <= (const state &s2) const {
    if (key.first < s2.key.first) return true;
    else if (key.first > s2.key.first) return false;
    return key.second < s2.key.second + 0.00001;
  }
  

  bool operator < (const state &s2) const {
    if (key.first + 0.000001 < s2.key.first) return true;
    else if (key.first - 0.000001 > s2.key.first) return false;
    return key.second < s2.key.second;
  }
};

class state_hash {
 public:
  size_t operator()(const state &s) const {
    return s.x + 34245*s.y;
  }
};

struct nodeInfo {
  double g;
  double rhs;
  double cost;
};

struct geometry_msgs_Point
{
    int x,y;
};

typedef priority_queue<state, vector<state>, greater<state> > PriorityQueue;
typedef hash_map<state, nodeInfo, state_hash, equal_to<state> > NodeHash;
typedef hash_map<state, float, state_hash, equal_to<state> > OpenHash;

PriorityQueue openList;
NodeHash nodeHash;
OpenHash openHash;

class DstarLite{

  public:
  // **TO DO: Store RobotID / openlist based on robotID in pathplanner to replan according to robotID
	 DstarLite(mygraph_t roadmap, WeightMap weightmap, geometry_msgs_Point* coordinates);
  // DstarLite();
	  virtual ~DstarLite();

/**
 * Initialize DstarLite.
 * @param start: the index number of start node
 * @param goal: the index number of goal node
 */
    void Initialize(Vertex start, Vertex goal);

/**
* Updates the costs for all nodes and computes the shortest path to goal (main() in [S. Koenig, 2002])
* @return true if it is successful, false otherwise
*/     
    bool Plan();   

  private:
        
    list<state> path;
    double km;
    int maxSteps;

    mygraph_t roadmap;
    WeightMap weightmap;
    geometry_msgs_Point* coordinates;

    state s_start, s_goal, s_last;

/**
  * Update vertex
  * @param u: state of the vertex that will be updated
  * @todo check UpdateVertex in optimized version of D*Lite as in [S. Koenig, 2002]
  * @todo check if deleting from priority queue needed (line 53)
  */ 

    void UpdateVertex(state u);
/**
  * Calculate shortest path as in [S. Koenig, 2002]
  * @return if 0 it is successful.
  * @todo Decide the maxsteps and give warning (line 69)
  */     
    int ComputeShortestPath();

/**
 * Calculate key of a node as in [S. Koenig, 2002]
 * @param u: node state
 * @return node state after calculating key
 */
    state  CalculateKey(state u);

/**
 * Calculate heuristic value between two nodes
 * @param a: node a
 * @param b: node b
 * @return heuristic value
 */
    double heuristic(state a, state b);

/**
 * Get a list of successor states for state u
 * @param u: node state
 * @param s: list of returned states
 */
    void getSucc(state u, list<state> &s);

/**
 * Get a list of predecessor states for state u
 * @param u: node state
 * @param s: list of returned states
 */
    void getPred(state u, list<state> &s);

/**
 * Set g value of a node
 * @param u: node
 * @param g: g value to be set
 */
    void setG(state u, double g);

/**
 * Set g value of a node
 * @param u: node
 * @param g: g value to be set
 */
    void setRHS(state u, double rhs);
  
/**
 * Get G value of a node.
 * @param u: node state
 * @return g value of the node
 */
    double getG(state u);

/**
 * Get RHS value of a node.
 * @param state: node state
 * @return rhs value of the node
 */
    double getRHS(state u); 

/**
 * Check if state u is on the open list 
 * @return true if state u is on the open list or not by checking if
 * it is in the open list.
 */
    bool isValid(state u);

/**
 * Check if the node is occupied
 * @return true if it is occupied
 */
    bool occupied(state u);

/**
 * Return the keyHashcode, which is used to compare a state that have been updated
 */
    float keyHashCode(state u);

/** 
 * Return the cost of traveling from node a to node b, calculated using edge infomation
 * @return cost
 * @todo use edge cost instead of coorinates (line 259)
 */
    double cost(state a, state b);

/**
 * Return if two numbers are close
 */
    bool close(double x, double y);

/**
 * Insert a node to priority queue
 */
    void insert(state u);
};
