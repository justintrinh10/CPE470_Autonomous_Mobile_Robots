import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
from point import Point
from path import Path
from line_segment import LineSegment
from visibility_graph import VisibilityGraph

num_points = 500
robot_radius = 0.18  # meters
dist_outside_box = 0.5  # meters

class ProcesssLidar(Node):
    def __init__(self):
        super().__init__("process_lidar")
        self.subscriber_ = self.create_subscription(
            String,
            "start_process_lidar",
            self.listener_callback,
            10,
        )

        self.publisher_ = self.create_publisher(String, "process_lidar_complete", 10)
        self.point_cloud = np.zeros((2, num_points))  # row 0: angle in degrees, row 1: distance in meters

    def listener_callback(self, msg):
        data_lines = msg.data.strip().split("\n")
        for i in range(len(data_lines)):
            angle_str, distance_str = data_lines[i].split(",")
            angle = float(angle_str)
            distance = float(distance_str)
            self.point_cloud[0, i] = angle
            self.point_cloud[1, i] = distance

        self.sort_points_by_angle()
        opening_point1, opening_point2 = self.find_largest_opening()
        start_point = Point(0.0, 0.0)
        end_point = self.find_point_outside_box(opening_point1, opening_point2, start_point)
        walls = self.find_walls()
        visibility_graph = VisibilityGraph(walls, start_point, end_point, robot_radius)
        path_points = visibility_graph.find_shortest_path_dijkstra()
        self.publish_path(path_points)

    def publish_path(self, path):
        path_msg = String()
        path_str = ""
        for point in path:
            path_str += f"{point.get_x()},{point.get_y()}\n"
        path_msg.data = path_str.strip()
        self.publisher_.publish(path_msg)
    
    def find_walls(self):
        breakpoints = self.segmented_least_squares()
        walls = []
        for i in range(len(breakpoints) - 1):
            pass


    def segmented_least_squares(self):
        breakpoints = []

        return breakpoints        

    def find_point_outside_box(self, opening_point1, opening_point2, origin):
        x1, y1 = opening_point1.get_x(), opening_point1.get_y()
        x2, y2 = opening_point2.get_x(), opening_point2.get_y()
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0

        vx = x2 - x1
        vy = y2 - y1
        nx = -vy
        ny = vx
        nmag = math.hypot(nx, ny)
        nx /= nmag
        ny /= nmag

        ox = mid_x - origin.get_x()
        oy = mid_y - origin.get_y()
        dot = nx * ox + ny * oy
        if dot < 0:
            nx = -nx
            ny = -ny
        dir_x = nx
        dir_y = ny

        offset = dist_outside_box + robot_radius
        target_x = mid_x + dir_x * offset
        target_y = mid_y + dir_y * offset

        outside_point = Point(target_x, target_y)

        return outside_point

    def find_largest_opening(self):
        self.sort_points_by_angle()
        max_distance = 0
        max_distance_index = 0
        prev_angle = self.point_cloud[0, 0]
        for i in range(1, num_points):
            cur_point = (self.point_cloud[0, i], self.point_cloud[1, i])
            prev_point = (self.point_cloud[0, i - 1], self.point_cloud[1, i - 1])
            distance_between_points = self.find_distance_between_points(cur_point, prev_point)
            if distance_between_points > max_distance:
                max_distance = distance_between_points
                max_distance_index = i
        last_point = (self.point_cloud[0, num_points - 1], self.point_cloud[1, num_points - 1])
        first_point = (self.point_cloud[0, 0], self.point_cloud[1, 0])
        wrap_distance = self.find_distance_between_points(first_point, last_point)
        if wrap_distance > max_distance:
            max_distance = wrap_distance
            max_distance_index = 0
        opening_point1 = Point.from_polar(self.point_cloud[0, max_distance_index - 1], self.point_cloud[1, max_distance_index - 1])
        opening_point2 = Point.from_polar(self.point_cloud[0, max_distance_index], self.point_cloud[1, max_distance_index])
        return opening_point1, opening_point2

    def find_distance_between_points(self, point1, point2):
        x1, y1 = self.polar_to_cartesian(point1)
        x2, y2 = self.polar_to_cartesian(point2)
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance
    
    def polar_to_cartesian(self, polar_point):
        angle_rad = math.radians(polar_point[0])
        x = polar_point[1] * math.cos(angle_rad)
        y = polar_point[1] * math.sin(angle_rad)
        return (x, y)
    
    def sort_points_by_angle(self):
        index = np.argsort(self.point_cloud[0])
        self.point_cloud[0] = self.point_cloud[0][index]
        self.point_cloud[1] = self.point_cloud[1][index]

    def least_square(self, data):
        H_matrix = self.create_jacobian_matrix(data)
        y_vec = self.create_y_vector(data)
        X_hat = self.least_square_equation(H_matrix, y_vec)
        return X_hat

    def least_square_equation(self,H, y):
        H_transposed = H.T
        temp1 = np.dot(H_transposed, H)
        temp1 = np.linalg.inv(temp1)
        temp2 = np.dot(H_transposed, y)
        return np.dot(temp1, temp2)

    def create_jacobian_matrix(self, data):
        jacobian_matrix = np.empty((len(data), 2))
        for i in range(len(data)):
            x = data[i].get_x()
            jacobian_matrix[i][0] = x
            jacobian_matrix[i][1] = 1
        return jacobian_matrix

    def create_y_vector(self, data):
        y_vector = np.empty((len(data), 1))
        for i in range(len(data)):
            y = data[i].get_y()
            y_vector[i][0] = y
        return y_vector
    
    def create_subset(self, data, point1, point2):
        data_subset = []

        def norm_angle_deg(a):
            return a % 360

        start_angle = norm_angle_deg(point1.get_angle())
        end_angle = norm_angle_deg(point2.get_angle())

        for p in data:
            try:
                cur_angle = norm_angle_deg(p.get_angle())
            except Exception:
                continue

            if start_angle <= end_angle:
                if start_angle <= cur_angle <= end_angle:
                    data_subset.append(p)
            else:
                if cur_angle >= start_angle or cur_angle <= end_angle:
                    data_subset.append(p)

        return data_subset

    def destroy_node(self):
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    process_lidar = ProcesssLidar()
    rclpy.spin(process_lidar)
    process_lidar.destroy_node()

if __name__ == "__main__":
    main()