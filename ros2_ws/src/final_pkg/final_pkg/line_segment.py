import math
import point as pt

class LineSegment:
    def __init__(self, point1, point2, slope, intercept):
        self.slope = slope
        self.intercept = intercept
        self.point1 = self.get_point_on_line(point1)
        self.point2 = self.get_point_on_line(point2)

    @classmethod
    def from_two_points(cls, point1, point2):
        slope = (point2.get_y() - point1.get_y()) / (point2.get_x() - point1.get_x())
        intercept = point1.get_y() - slope * point1.get_x()
        return cls(point1, point2, slope, intercept)

    def distance(self, p1, p2):
        return math.sqrt((p2.get_x() - p1.get_x())**2 + (p2.get_y() - p1.get_y())**2)

    def get_point_on_line(self, point):
        perp_slope = -1 / self.slope
        perp_intercept = point.get_y() - perp_slope * point.get_x()

        x = (perp_intercept - self.intercept) / (self.slope - perp_slope)
        y = self.slope * x + self.intercept

        return pt.Point.from_cartesian(x, y)

    def get_distance_to_point(self, point):
        proj = self.get_point_on_line(point)
        if not self.point_within_segment(proj):
            return 1000
        return self.distance(point, proj)

    def point_within_segment(self, p):
        x = p.get_x()
        y = p.get_y()
        if min(self.point1.get_x(), self.point2.get_x()) <= x <= max(self.point1.get_x(), self.point2.get_x()):
            if min(self.point1.get_y(), self.point2.get_y()) <= y <= max(self.point1.get_y(), self.point2.get_y()):
                return True
        return False

    def intersect_with_line_segment(self, other):
        m1, b1 = self.slope, self.intercept
        m2, b2 = other.slope, other.intercept

        if m1 == m2:
            return False

        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1
        point = pt.Point(x, y)

        if self.point_within_segment(point) and other.point_within_segment(point):
            return True

        return False
    
    def get_minkowski_sum(self, robot_radius):
        x1, y1 = self.point1.get_x(), self.point1.get_y()
        x2, y2 = self.point2.get_x(), self.point2.get_y()
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        if length == 0:
            raise ValueError("Wall segment has zero length")
        nx = dx / length
        ny = dy / length
        px = -ny
        py = nx

        point3  = pt.Point(x1 + robot_radius * px, y1 + robot_radius * py)
        point4 = pt.Point(x1 - robot_radius * px, y1 - robot_radius * py)
        point5  = pt.Point(x2 + robot_radius * px, y2 + robot_radius * py)
        point6 = pt.Point(x2 - robot_radius * px, y2 - robot_radius * py)

        return point3, point4, point5, point6


    def intersect_with_minkowski_sum(self, other, robot_radius):
        point3, point4, point5, point6 = other.get_minkowski_sum(robot_radius)

        line1 = LineSegment.from_two_points(point3, point4)
        line2 = LineSegment.from_two_points(point5, point6)
        line3 = LineSegment.from_two_points(point3, point5)
        line4 = LineSegment.from_two_points(point4, point6)

        if line1.intersect_with_line_segment(self):
            return True
        if line2.intersect_with_line_segment(self):
            return True
        if line3.intersect_with_line_segment(self):
            return True
        if line4.intersect_with_line_segment(self):
            return True
        
        return False