import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @classmethod
    def from_polar(cls, angle_deg, distance):
        a = math.radians(angle_deg)
        x = distance * math.cos(a)
        y = distance * math.sin(a)
        return cls(x, y)

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
    
    def get_angle(self):
        return math.degrees(math.atan2(self.y, self.x))
    
    def get_distance(self):
        return math.sqrt(self.x**2 + self.y**2)