import math

start_angle = 90

class Path:
    def __init__(self, points):
        self.points = points
        self.distances = []
        self.angle_changes = []
        self.calculate_distances()
        self.calculate_angles()
    
    def calculate_distances(self):
        for i in range(1, len(self.points)):
            p1 = self.points[i - 1]
            p2 = self.points[i]
            distance = self.calc_distance(p1, p2)
            self.distances.append(distance)
    
    def calc_distance(self, p1, p2):
        return math.sqrt((p2.get_x() - p1.get_x())**2 + (p2.get_y() - p1.get_y())**2)
    
    def calculate_angles(self):
        current_angle = start_angle  

        if len(self.points) >= 2:
            p1 = self.points[0]
            p2 = self.points[1]
            first_seg_angle = math.degrees(math.atan2(p2.get_y() - p1.get_y(), p2.get_x() - p1.get_x()))

            first_turn = (first_seg_angle - current_angle + 180) % 360 - 180
            self.angle_changes.append(first_turn)
            current_angle = first_seg_angle

        for i in range(1, len(self.points) - 1):
            p1 = self.points[i]
            p2 = self.points[i + 1]

            next_seg_angle = math.degrees(math.atan2(p2.get_y() - p1.get_y(), p2.get_x() - p1.get_x()))
            turn = (next_seg_angle - current_angle + 180) % 360 - 180

            self.angle_changes.append(turn)
            current_angle = next_seg_angle

