import math
import point as pt
import line_segment as ls

class VisibilityGraph:
    def __init__(self, walls, start, goal, robot_radius):
        self.walls = walls
        self.start = start
        self.goal = goal
        self.robot_radius = robot_radius
        self.visibility_graph = self.create_visibility_graph()

    def create_visibility_graph(self):
        pass

    def is_visible(self, p1, p2):
        
        pass

    def find_shortest_path_dijkstra(self):
        pass