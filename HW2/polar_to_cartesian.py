import math

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

DATA_FILES = ["lidar1.txt", "lidar2.txt"]

def main():
    data = []
    for file_name in DATA_FILES:
        data.append(read_file(file_name))
    for i in range(len(data)):
        print(f"Data Set {i + 1}")
        print("(Angle[degrees], Range[cm]) (x[cm], y[cm])")
        for measurement in data[i]:
            measurement.output()
        print()

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