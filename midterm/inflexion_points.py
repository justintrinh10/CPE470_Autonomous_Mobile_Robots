import polar_to_cartesian as ptc
import math
import numpy
import matplotlib.pyplot as plt

DATA_FILE = "lidar_data.csv"
ERROR = 20
THRESHOLD = 2

def find_inflexion_points(data):
    second_derivatives = find_second_derivative(data)
    inflexion_points_ind = []
    for i in range(len(second_derivatives)):
        if abs(second_derivatives[i]) > THRESHOLD:
            inflexion_points_ind.append(i)
    return inflexion_points_ind

def find_first_derivative(data):
    f_derv = []
    for i in range(len(data)):
        cur_angle, cur_dist = data[(i + 1)%len(data)].getPolar()
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
        cur_angle, trash = data[(i + 1)%len(first_derivatives)].getPolar()
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

#window must be odd
def create_average_data_set(data, window):
    avg_data_set = []
    half_window = int(window/2)
    for i in range(len(data)):
        total_angle = 0
        total_dist = 0
        for j in range(i - half_window, i + half_window):
            angle, dist = data[j%len(data)].getPolar()
            total_angle += angle
            total_dist += dist
        angle_avg = total_angle/window
        dist_avg = total_dist/window
        measurement = ptc.Measurement(angle_avg, dist_avg)
        avg_data_set.append(measurement)
    return avg_data_set

def get_inflexion_points(data, ind):
    inflexion_points = []
    for i in ind:
        inflexion_points.append(data[i])
    return inflexion_points

def main():
    data = ptc.read_file(DATA_FILE)
    avg_data_set = create_average_data_set(data, 11)
    inflexion_points_ind = find_inflexion_points(avg_data_set)
    inflexion_points = get_inflexion_points(data, inflexion_points_ind)
    ptc.configure_scatter_plot()
    ptc.add_data_scatter(data, 1, "blue")
    ptc.add_data_scatter(inflexion_points, 10, "red")
    plt.show()

    # avg_data_set = create_average_data_set(data,11)
    # second_derivatives = find_second_derivative(avg_data_set)
    # x_axis = list(range(len(second_derivatives)))
    # plt.scatter(x_axis, second_derivatives, s=1)
    # plt.xlabel("Data Points")
    # plt.ylabel("Second Derivatives")
    # plt.show()

if __name__ == "__main__":
    main()