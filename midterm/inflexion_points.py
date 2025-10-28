import polar_to_cartesian as ptc
import math
import numpy
import matplotlib.pyplot as plt

DATA_FILE = "lidar_data.csv"
ERROR = 20
THRESHOLD = 100000

def find_inflexion_points(data):
    second_derivatives = find_second_derivative(data)
    inflexion_points = []
    for i in range(len(second_derivatives)):
        if abs(second_derivatives[i]) > THRESHOLD:
            inflexion_points.append(data[i])
    return inflexion_points

def find_first_derivative(data):
    f_derv = []
    for i in range(len(data)):
        cur_angle, cur_dist = data[i].getPolar()
        prev_angle, prev_dist = data[i - 1].getPolar()
        derivative = 0
        if cur_angle == prev_angle:
            derivative = f_derv[i - 1]
        else:
            angle_diff = (cur_angle - prev_angle + 360) % 360
            if angle_diff > 180:
                angle_diff -= 360
            derivative = (cur_dist - prev_dist)/angle_diff
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
            angle_diff = (cur_angle - prev_angle + 360) % 360
            if angle_diff > 180:
                angle_diff -= 360
            second_derivative = (first_derivatives[i] - first_derivatives[i - 1])/angle_diff
        s_derv.append(second_derivative)
    return s_derv

def main():
    data = ptc.read_file(DATA_FILE)
    inflexion_points = find_inflexion_points(data)
    ptc.configure_scatter_plot()
    ptc.add_data_scatter(data, 1, "blue")
    ptc.add_data_scatter(inflexion_points, 10, "red")
    plt.show()

    # second_derivatives = find_second_derivative(data)
    # x_axis = list(range(len(second_derivatives)))
    # plt.scatter(x_axis, second_derivatives, s=1)
    # plt.xlabel("Data Points")
    # plt.ylabel("Second Derivatives")
    # plt.show()

if __name__ == "__main__":
    main()