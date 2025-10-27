import polar_to_cartesian as ptc
import math
import numpy
import matplotlib.pyplot as plt

DATA_FILE = "lidar_data.csv"
ERROR = 20

#def find_inflexion_points(data):

def find_first_derivative(data):
    f_derv = []
    for i in range(len(data)):
        cur_angle, cur_dist = data[i].getPolar()
        prev_angle, prev_dist = data[i - 1].getPolar()
        derivative = 0
        if cur_angle == prev_angle:
            derivative = f_derv[i - 1]
        else:
            derivative = (cur_dist - prev_dist)/(cur_angle - prev_angle)
        f_derv.append(derivative)
    return f_derv

def find_second_derivative(data):
    first_derivatives = find_first_derivative(data)
    s_derv = []
    for i in range(len(first_derivatives)):
        cur_angle, trash = data[i].getPolar()
        prev_angle, trash = data[i - 1].getPolar()
        second_derivative = 0
        if cur_angle == prev_angle:
            second_derivative = s_derv[i - 1]
        else:
            second_derivative = (first_derivatives[i] - first_derivatives[i - 1])/(cur_angle - prev_angle)
        s_derv.append(second_derivative)
    return s_derv

def main():
    data = ptc.read_file(DATA_FILE)
    second_derivatives = find_second_derivative(data)
    x_axis = list(range(len(second_derivatives)))
    plt.scatter(x_axis, second_derivatives, s=1)
    plt.show()

if __name__ == "__main__":
    main()