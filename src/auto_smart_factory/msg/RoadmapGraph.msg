# Contains the graph created by Roadmap Generator and the corresponding coordinates.

# node names are assumed to be natural numbers from 0 to num_nodes
int32 num_nodes
int32 num_edges

# coordinates that correspond to nodes
geometry_msgs/Point[] coordinates

# integer coordinates correspond to real coordinates but grid nodes have distance 1
int32[] x_coord_int
int32[] y_coord_int

# resolution is the distance between two grid nodes
float64 resolution
float64 height
float64 width

# edges are given as two arrays, third array contains the length (in real value, not int)
int32[] start_nodes
int32[] end_nodes
float64[] lengths

# contains value True if edge is grid edge, False otherwise
bool[] is_grid_edge

# start_nodes.length() == end_nodes.length() == lengths.length() == is_grid_edge.length() must always be true
 


