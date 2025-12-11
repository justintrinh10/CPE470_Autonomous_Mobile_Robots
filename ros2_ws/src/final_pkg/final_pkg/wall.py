import math

class LineSegment:
    def __init__(self, point1, point2, slope, intercept):
        self.slope = slope
        self.intercept = intercept
        self.point1 = self.get_point_on_line(point1)
        self.point2 = self.get_point_on_line(point2)

    def from_two_points(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
        self.slope = (point2.get_y() - point1.get_y()) / (point2.get_x() - point1.get_x())
        self.intercept = point1.get_y() - self.slope * point1.get_x()
        return self

    def distance(self, p1, p2):
        return math.sqrt((p2.get_x() - p1.get_x())**2 + (p2.get_y() - p1.get_y())**2)

    def get_point_on_line(self, point):
        perp_slope = -1 / self.slope
        perp_intercept = point.get_y() - perp_slope * point.get_x()

        x = (perp_intercept - self.intercept) / (self.slope - perp_slope)
        y = self.slope * x + self.intercept

        return Point.from_cartesian(x, y)

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
        pt = Point.from_cartesian(x, y)

        if self.point_within_segment(pt) and other.point_within_segment(pt):
            return True

        return False

    def intersect_with_minkowski_sum(self, other, robot_radius):
        if self.intersect_with_line_segment(other):
            return True

        dist1 = self.get_distance_to_point(other.point1)
        dist2 = self.get_distance_to_point(other.point2)
        dist3 = other.get_distance_to_point(self.point1)
        dist4 = other.get_distance_to_point(self.point2)

        return (dist1 < robot_radius or dist2 < robot_radius or dist3 < robot_radius or dist4 < robot_radius)

class Point:
    def __init__(self, angle, distance):
        self.x, self.y = self.polar_to_cartesian(angle, distance)

    def from_cartesian(self, x, y):
        self.x = x
        self.y = y
        return self

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
    
    def get_angle(self):
        return math.degrees(math.atan2(self.y, self.x))
    
    def get_distance(self):
        return math.sqrt(self.x**2 + self.y**2)