import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

num_points = 360

class FindOpening(Node):
    def __init__(self):
        super().__init__("find_opening")
        self.subscriber_ = self.create_subscription(
            String,
            "lidarDataPolar",
            self.listener_callback,
            10,
        )
        self.publisher_ = self.create_publisher(String, "openingData", 10)
        self.point_cloud = np.zeros((2, num_points))  # row 0: angle in degrees, row 1: distance in mm

    def listener_callback(self, msg):
        data_lines = msg.data.strip().split("\n")
        for i in range(len(data_lines)):
            angle_str, distance_str = data_lines[i].split(",")
            angle = float(angle_str)
            distance = float(distance_str)
            self.point_cloud[0, i] = angle
            self.point_cloud[1, i] = distance
        self.find_largest_opening()
    
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
        opening_point1 = (self.point_cloud[0, max_distance_index - 1], self.point_cloud[1, max_distance_index - 1])
        opening_point2 = (self.point_cloud[0, max_distance_index], self.point_cloud[1, max_distance_index])
        opening_point1_cartesian = self.polar_to_cartesian(opening_point1)
        opening_point2_cartesian = self.polar_to_cartesian(opening_point2)
        msg = String()
        msg.data += f"{opening_point1_cartesian[0]:.2f},{opening_point1_cartesian[1]:.2f}\n"
        msg.data += f"{opening_point2_cartesian[0]:.2f},{opening_point2_cartesian[1]:.2f}\n"
        self.publisher_.publish(msg)
        self.get_logger().info("Published Opening Data")
        self.get_logger().info("Opening Points (Cartesian):")
        self.get_logger().info(f"Point 1: x = {opening_point1_cartesian[0]:.2f} mm, y = {opening_point1_cartesian[1]:.2f} mm")
        self.get_logger().info(f"Point 2: x = {opening_point2_cartesian[0]:.2f} mm, y = {opening_point2_cartesian[1]:.2f} mm")
    
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

    def destroy_node(self):
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    findOpeningNode = FindOpening()
    rclpy.spin(findOpeningNode)
    findOpeningNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()