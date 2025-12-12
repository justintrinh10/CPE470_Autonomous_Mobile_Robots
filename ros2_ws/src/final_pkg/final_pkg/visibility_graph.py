import math
import networkx as nx
import point as pt
import line_segment as ls

class VisibilityGraph:
    def __init__(self, walls, start, goal, robot_radius):
        self.walls = walls
        self.start = start
        self.goal = goal
        self.robot_radius = robot_radius
        self.visibility_graph = nx.Graph()
        self.create_visibility_graph()

    def create_visibility_graph(self):
        self.create_nodes()
        self.create_edges()

    def create_nodes(self):
        self.visibility_graph.add_node(self.start)
        self.visibility_graph.add_node(self.goal)
        for wall in self.walls:
            point3, point4, point5, point6 = wall.get_minkowski_sum(self.robot_radius)
            self.visibility_graph.add_node(point3)
            self.visibility_graph.add_node(point4)
            self.visibility_graph.add_node(point5)
            self.visibility_graph.add_node(point6)
    
    def create_edges(self):
        nodes = list(self.visibility_graph.nodes)
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                p1 = nodes[i]
                p2 = nodes[j]
                if self.is_visible(p1, p2):
                    distance = self.get_distance(p1, p2)
                    self.visibility_graph.add_edge(p1, p2, weight=distance)

    def is_visible(self, p1, p2):
        line = ls.LineSegment(p1, p2)
        for wall in self.walls:
            if line.intersect_with_minkowski_sum(wall):
                return False
        return True

    def get_distance(self, p1, p2):
        distance = math.hypot(p2.get_x() - p1.get_x(), p2.get_y() - p1.get_y())
        return distance

    def find_shortest_path_dijkstra(self):
        pass