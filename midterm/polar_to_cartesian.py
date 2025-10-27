import math
import matplotlib.pyplot as plt

class Measurement:
    def __init__(self, a, d):
        self.angle = a
        self.dist = d
        self.x, self.y = self.polar_to_cartesian(self.angle, self.dist)

    def polar_to_cartesian(self, a, d):
        r = math.radians(a)
        x = d * math.cos(r)
        y = d * math.sin(r)
        return x, y
    
    def output(self):
        print(f"({self.angle:.2f}, {self.dist:.2f}) ({self.x:.2f}, {self.y:.2f})")
    
    def getCartesian(self):
        return self.x, self.y
    
    def getPolar(self):
        return self.angle, self.dist
    
    def getLidar(self):
        return self.lidar

DATA_FILE = "lidar_data.csv"

def main():
    data = read_file(DATA_FILE)

    print(f"Data Set")
    print("(Angle[degrees], Range[cm]) (x[cm], y[cm])")
    x = []
    y = []
    for data_point in data:
        data_point.output()
        x_cord, y_cord = data_point.getCartesian()
        x.append(x_cord)
        y.append(y_cord)

    plt.scatter(x, y, s=1)
    plt.title("Cartesian Plot for Lidar Data")
    plt.xlabel("X-Axis [cm]")
    plt.ylabel("Y-Axis [cm]")
    plt.axis('equal')
    plt.show()

def read_file(file_name):
    measurements = [] 
    with open(file_name, 'r') as file:
        file.readline()
        for line in file:
            line = line.strip()
            if not line:
                continue
            items = line.split(',')
            angle = float(items[0])
            dist = float(items[1])
            measurements.append(Measurement(angle, dist))
    return measurements

if __name__ == "__main__":
    main()