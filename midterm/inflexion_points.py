import polar_to_cartesian as ptc
import math
import numpy as np
import matplotlib.pyplot as plt

DATA_FILE = "lidar_data.csv"
ERROR = 20
THRESHOLD = 15

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
            if i > 0:
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
            if i > 0:
                second_derivative = s_derv[i - 1]
        else:
            angle_diff = (cur_angle - prev_angle + 360) % 360
            if angle_diff > 180:
                angle_diff -= 360
            second_derivative = (first_derivatives[(i + 1)%len(first_derivatives)] - first_derivatives[i - 1])/angle_diff
        s_derv.append(second_derivative)
    return s_derv

#window must be odd && drop < window
def create_average_data_set(data, window, drop):
    avg_data_set = []
    half_window = int(window/2)
    for i in range(len(data)):
        angles = []
        dists = []
        for j in range(i - half_window, i + half_window):
            angles.append(data[j%len(data)].getPolar()[0])
            dists.append(data[j%len(data)].getPolar()[1])
        angles = np.array(angles)
        dists = np.array(dists)
        median_dist = np.median(dists)
        errors_from_median = np.empty((len(dists)))
        for j in range(len(dists)):
            errors_from_median[j] = abs(dists[j] - median_dist)
        for j in range(drop):
            max_error_index = np.argmax(errors_from_median)
            errors_from_median = np.delete(errors_from_median, max_error_index)
            dists = np.delete(dists, max_error_index)
            angles = np.delete(angles, max_error_index)
        avg_angle = np.mean(angles)
        avg_dist = np.mean(dists)
        measurement = ptc.Measurement(avg_angle, avg_dist)
        avg_data_set.append(measurement)
    return avg_data_set

def get_inflexion_points(data, ind):
    inflexion_points = []
    for i in ind:
        inflexion_points.append(data[i])
    return inflexion_points

def find_wall_ends(data):
    max_angle_gap = 0
    max_angle_gap_index = 0
    for i in range(len(data)):
        cur_angle = data[i].getPolar()[0]
        prev_angle = data[i - 1].getPolar()[0]
        if cur_angle - prev_angle > max_angle_gap:
            max_angle_gap = cur_angle - prev_angle
            max_angle_gap_index = i
    return data[max_angle_gap_index - 1], data[max_angle_gap_index]

def main():
    data = ptc.read_file(DATA_FILE)
    avg_data_set = create_average_data_set(data, 11, 4)
    inflexion_points_ind = find_inflexion_points(avg_data_set)
    inflexion_points = get_inflexion_points(data, inflexion_points_ind)
    ptc.configure_scatter_plot()
    ptc.add_data_scatter(data, 1, "blue")
    ptc.add_data_scatter(inflexion_points, 10, "red")
    wall_end_points = find_wall_ends(data)
    ptc.add_data_scatter(wall_end_points, 20, "orange")
    plt.show()

    # avg_data_set = create_average_data_set(data,15, 5)
    # second_derivatives = find_second_derivative(avg_data_set)
    # x_axis = list(range(len(second_derivatives)))
    # plt.scatter(x_axis, second_derivatives, s=1)
    # plt.xlabel("Data Points")
    # plt.ylabel("Second Derivatives")
    # plt.show()

if __name__ == "__main__":
    main()