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
        self.threshold = 1.75
        self.window_size = 21
        self.drop_size = 7
        self.opening_point1 = None
        self.opening_point2 = None

    def listener_callback(self, msg):
        data_lines = msg.data.strip().split("\n")
        for i in range(len(data_lines)):
            angle_str, distance_str = data_lines[i].split(",")
            angle = float(angle_str)
            distance = float(distance_str)
            self.point_cloud[0, i] = angle
            self.point_cloud[1, i] = distance

        self.sort_points_by_angle()
        self.opening_point1, self.opening_point2 = self.find_largest_opening()
        start_point = Point(0.0, 0.0)
        end_point = self.find_point_outside_box(self.opening_point1, self.opening_point2, start_point)
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
        points = self.get_points_from_data(self.point_cloud.T)
        avg_points = self.create_average_data_set(points, self.window_size, self.drop_size)
        corners = self.find_inflexion_points(avg_points)
        corners.append(self.opening_point1)
        corners.append(self.opening_point2)
        corners.sort()
        walls = []
        for i in range(len(corners)):
            point1 = corners[i]
            point2 = corners[(i + 1) % len(corners)]
            if (point1 == self.opening_point1 and point2 == self.opening_point2) or (point1 == self.opening_point2 and point2 == self.opening_point1):
                continue
            subset = self.create_subset(points, point1, point2)
            x_hat = self.least_square(subset)
            slope = x_hat[0][0]
            intercept = x_hat[1][0]
            wall = LineSegment(point1, point2, slope, intercept)
            walls.append(wall)
        return walls

    def find_inflexion_points(self, data):
        second_derivatives = self.find_second_derivative(data)
        first_derivative = self.find_first_derivative(data)
        possible_inflexion_points_ind = []
        for i in range(1, len(first_derivative) - 1):
            cur_derivative = first_derivative[i]
            next_derivative = first_derivative[i + 1]
            if cur_derivative > 0 and next_derivative < 0:
                possible_inflexion_points_ind.append(i)
        inflexion_points_ind = []
        for i in range(len(possible_inflexion_points_ind)):
            if second_derivatives[possible_inflexion_points_ind[i] + 1] < -self.threshold:
                inflexion_points_ind.append(possible_inflexion_points_ind[i])
        inflexion_points = self.get_inflexion_points(data, inflexion_points_ind)
        return inflexion_points

    def find_first_derivative(self, data):
        f_derv = []
        for i in range(len(data)):
            cur_angle = data[i].get_angle()
            cur_dist = data[i].get_distance()
            prev_angle = data[i - 1].get_angle()
            prev_dist = data[i - 1].get_distance()
            derivative = 0
            if cur_angle == prev_angle:
                if i > 0:
                    derivative = f_derv[i - 1]
            #ignore large gaps in data
            elif cur_angle - prev_angle > 5:
                derivative = 0
            else:
                angle_diff = (cur_angle - prev_angle + 360) % 360
                if angle_diff > 180:
                    angle_diff -= 360
                derivative = (cur_dist - prev_dist)/angle_diff
            f_derv.append(derivative)
        return f_derv

    def find_second_derivative(self, data):
        first_derivatives = self.find_first_derivative(data)
        s_derv = []
        for i in range(len(first_derivatives)):
            cur_angle = data[i].get_angle()
            prev_angle = data[i - 1].get_angle()
            second_derivative = 0
            if cur_angle == prev_angle:
                if i > 0:
                    second_derivative = s_derv[i - 1]
            else:
                angle_diff = (cur_angle - prev_angle + 360) % 360
                if angle_diff > 180:
                    angle_diff -= 360
                second_derivative = (first_derivatives[i] - first_derivatives[i - 1])/angle_diff
            s_derv.append(second_derivative)
        return s_derv

    #window must be odd && drop < window
    def create_average_data_set(self, data, window, drop):
        avg_data_set = []
        half_window = int(window/2)
        for i in range(0, len(data), 10):
            angles = []
            dists = []
            for j in range(i - half_window, i + half_window):
                angles.append(data[j%len(data)].get_angle())
                dists.append(data[j%len(data)].get_distance())
            angles = np.array(angles)
            dists = np.array(dists)
            median_dist = np.median(dists)
            errors_from_median = np.empty((len(dists)))
            for j in range(len(dists)):
                errors_from_median[j] = abs(dists[j] - median_dist)
            for j in range(drop):
                max_error_index = np.argmax(errors_from_median)
                errors_from_median = np.delete(errors_from_median, max_error_index)
                dists = np.delete(dists, max_error_index)
                angles = np.delete(angles, max_error_index)
            angle_total = 0
            for angle in angles:
                angle_total += ((angle - angles[0] + 180) % 360) - 180
            avg_angle = (angles[0] + angle_total/len(angles)) % 360
            avg_dist = np.mean(dists)
            point = Point.from_polar(avg_angle, avg_dist)
            avg_data_set.append(point)
        avg_data_set.sort()
        return avg_data_set

    def get_inflexion_points(self, data, ind):
        inflexion_points = []
        for i in ind:
            inflexion_points.append(data[i])
        return inflexion_points
    
    def get_points_from_data(self, data):
        points = []
        for i in range(len(data)):
            angle = data[i][0]
            dist = data[i][1]
            point = Point.from_polar(angle, dist)
            points.append(point)
        return points
    
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
    
    def create_subset(data, point1, point2):
        data_subset = []
        start_angle = point1.get_angle()
        end_angle = point2.get_angle()
        if start_angle <= end_angle:
            for i in range(len(data)):
                cur_angle = data[i].get_angle()
                if start_angle <= cur_angle and cur_angle <= end_angle:
                    data_subset.append(data[i])
        else:
            for i in range(len(data)):
                cur_angle = data[i].get_angle()
                if cur_angle >= start_angle or cur_angle <= end_angle:
                    data_subset.append(data[i])
        return data_subset

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

    def destroy_node(self):
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    process_lidar = ProcesssLidar()
    rclpy.spin(process_lidar)
    process_lidar.destroy_node()

if __name__ == "__main__":
    main()