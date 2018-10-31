#!/usr/bin/env python3

"""
Creates a grid-like graph (containing also diagonals) which is used for all
tasks related to path planning for the robots. The graph generation assumes
that all static obstacles have a convex shape. Otherwise the resulting graph
may be non-connected. The graph covers the whole factory with equally sized
cells, a margin from all obstacles is kept free of nodes.

@author Ansgar
"""

import rospy
import networkx as nx
from geometry_msgs.msg import Point, Pose
from auto_smart_factory.msg import RoadmapGraph
from auto_smart_factory.srv import *
import matplotlib.pyplot as plt   # can be used to visualize the roadmap graph
from math import gcd, sqrt  # gcd needs Python 3.5



def createPoint(x, y):
    """Utility function to create instances of geometry_msgs/Point."""
    p = Point()
    p.x = x
    p.y = y
    return p


class RoadmapGenerator:
    """
    Builds and holds the graph which is used as a roadmap.
    """

    def __init__(self):
        """The parameter self.margin adjusts how close to static obstacles
        the nodes of the graph are created. In the first place, all nodes
        are scattered equally over the map leaving aside the static obstacles.
        Then the closest self.margin number of nodes to any obstacle are
        removed. Therefore, self.margin should be adjusted to the resolution
        and the size of the robots, e.g. margin = robot_diameter / resolution."""

        self.graph = nx.Graph()
        self.coordinates = []
        self.is_ready = False
        self.margin = 2    # margin around obstacles, given as number of nodes / unit edges distance


    def buildNewRoadmap(self, req):
        """
        Service handler for trigger_roadmap_generator service.

        Args:
            req: triggerRoadmapGenerator service request data

        Returns:
            bool: indicates successful creation of roadmap
        """

        self.is_ready = False
        self.graph = nx.Graph()
        self.coordinates = []
        self.height = req.warehouse_configuration.height
        self.width = req.warehouse_configuration.width


        occ_map = req.warehouse_configuration.occupancy_map
        self.occ_grid = occ_map.data
        _info = occ_map.info  # has fields .resolution, .width, .height, .origin (-> Pose), map_load_time
        self.resolution = _info.resolution
        rospy.loginfo("resolution " + str(self.resolution))


        # basically we use the occupancy grid as our graph
        self.int_coordinates = [(j, i) for i in range(_info.height) for j in range(_info.width)]
        self.real_coordinates = [(a * self.resolution, b * self.resolution) for a, b in self.int_coordinates]
        self.int_coord_dict = {c: i for i, c in enumerate(self.int_coordinates)}
        self.graph.add_nodes_from(zip(range(_info.width * _info.height),
                                      ({"coord": c, "coord_int": d} for c, d
                                       in zip(self.real_coordinates, self.int_coordinates))))

        # Edge Generators
        def edge_h_gen():
            for i in range(_info.height):
                for j in range(_info.width - 1):
                    a, b = i * _info.width + j, i * _info.width + j + 1
                    if self.occ_grid[a] == 0 and self.occ_grid[b] == 0:
                        yield (a, b, {"length": self.resolution, "is_grid": True})

        def edge_v_gen():
            for i in range(_info.height - 1):
                for j in range(_info.width):
                    a, b = i * _info.width + j, (i + 1) * _info.width + j
                    if self.occ_grid[a] == 0 and self.occ_grid[b] == 0:
                        yield (a, b, {"length": self.resolution, "is_grid": True})

        def edge_diag_up_gen(k):
            """Generate upward diagonals whose projection in x or y direction has length at most k."""
            def blocked(i, j, d, e):
                """Exclude upward diagonals through obstacles."""
                return any(self.occ_grid[_i * _info.width + _j] for _i in range(i, i + e + 1) for _j in range(j, j + d + 1))
            for d in range(1, k + 1):
                for e in range(1, d + 1):
                    if gcd(e, d) == 1:  # avoid overlaying diagonals

                        for i in range(_info.height - e):
                            for j in range(_info.width - d):
                                a, b = i * _info.width + j, (i + e) * _info.width + j + d
                                if not blocked(i, j, d, e):
                                    yield (a, b, {"length": self.resolution*sqrt(e**2 + d**2), "is_grid": False})

                        for i in range(_info.height - d):
                            for j in range(_info.width - e):
                                a, b = i * _info.width + j, (i + d) * _info.width + j + e
                                if not blocked(i, j, e, d):
                                    yield (a, b, {"length": self.resolution * sqrt(e ** 2 + d ** 2), "is_grid": False})

        def edge_diag_down_gen(k):
            """Generate downward diagonals whose projection in x or y direction has length at most k."""
            def blocked(i, j, d, e):
                """Exclude downward diagonals through obstacles."""
                return any(self.occ_grid[_i * _info.width + _j] for _i in range(i, i + e + 1) for _j in range(j, j - d - 1, -1))
            for d in range(1, k + 1):
                for e in range(1, d + 1):
                    if gcd(e, d) == 1:  # avoid overlaying diagonals

                        for i in range(_info.height - e):
                            for j in range(d, _info.width):
                                a, b = i * _info.width + j, (i + e) * _info.width + j - d
                                if not blocked(i, j, d, e):
                                    yield (a, b, {"length": self.resolution * sqrt(e ** 2 + d ** 2), "is_grid": False})

                        for i in range(_info.height - d):
                            for j in range(e, _info.width):
                                a, b = i * _info.width + j, (i + d) * _info.width + j - e
                                if not blocked(i, j, e, d):
                                    yield (a, b, {"length": self.resolution * sqrt(e ** 2 + d ** 2), "is_grid": False})

        self.graph.add_edges_from(edge_v_gen())
        self.graph.add_edges_from(edge_h_gen())
        self.graph.add_edges_from(edge_diag_down_gen(2))   # use diagonals up to a length of two nodes
        self.graph.add_edges_from(edge_diag_up_gen(2))

        # remove nodes which are on static obstacles or close to static obstacles
        old_node_num = self.graph.number_of_nodes()
        remove_nodes = []
        for n in range(old_node_num):
            x, y = self.int_coordinates[n]
            for i in range(-self.margin, self.margin + 1):
                for j in range(-self.margin, self.margin + 1):
                    try:
                        if self.occ_grid[self.int_coord_dict[x + i, y + j]]:
                            remove_nodes.append(n)
                    except KeyError:
                        pass    # also remove nodes that are close to the border of the factory

        if self.margin > 1:
            self.margin -= 1   # smaller margin at factory borders
        for x, y in self.int_coordinates:
            if x < self.margin or x >= self.width / self.resolution - self.margin:
                remove_nodes.append(self.int_coord_dict[x, y])
            if y < self.margin or y >= self.height / self.resolution - self.margin:
                remove_nodes.append((self.int_coord_dict[x, y]))
        self.graph.remove_nodes_from(remove_nodes)
        self.graph = nx.convert_node_labels_to_integers(self.graph)


        # Plot the graph on screen. Requires matplotlib. Should not be more than ~ 1000 nodes.
        #nx.draw_networkx(self.graph, pos={n: co for n, co in self.graph.nodes(data="coord")})
        #plt.show()


        self.is_ready = True
        return True  # returns to the service that building the roadmap was successful.

    def buildRoadmapMessage(self):
        """
        Constructs a message which can be published by the roadmap_graph publisher.

        Returns:
            RoadmapGraph: message to be published
        """

        msg = RoadmapGraph()
        msg.num_nodes = self.graph.number_of_nodes()
        msg.num_edges = self.graph.number_of_edges()
        msg.coordinates = [createPoint(x, y) for _, (x, y) in self.graph.nodes(data="coord")]
        msg.x_coord_int, msg.y_coord_int = zip(*((x, y) for _, (x, y) in self.graph.nodes(data="coord_int")))

        msg.height = self.height
        msg.width = self.width
        msg.resolution = self.resolution

        try:
            msg.start_nodes, msg.end_nodes, msg.lengths = zip(*self.graph.edges(data="length"))
            _, __, msg.is_grid_edge = zip(*self.graph.edges(data="is_grid"))
        except ValueError:
            rospy.logerr("Roadmap graph should be published but contains no edges.")
        return msg

if __name__ == "__main__":
    '''
    On startup the node publishes one RoadmapGraph message. This can be used by path and traffic planners
    '''
    rospy.init_node("roadmap_generator")
    publisher = rospy.Publisher("roadmap_graph", RoadmapGraph, queue_size=5)

    rg = RoadmapGenerator()
    r = rospy.Rate(0.3)
    rospy.Service("roadmap_generator/trigger_roadmap_generator", triggerRoadmapGenerator, rg.buildNewRoadmap)
    while not rg.is_ready:
        r.sleep()

    publisher.publish(rg.buildRoadmapMessage())
    rospy.loginfo("RoadmapGenerator published RoadmapGraph message")
    rospy.spin()
