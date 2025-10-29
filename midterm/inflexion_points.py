import polar_to_cartesian as ptc
import math
import numpy as np
import matplotlib.pyplot as plt

DATA_FILE = "lidar_data.csv"
ERROR = 20
THRESHOLD = 1.75

def find_inflexion_points(data):
    second_derivatives = find_second_derivative(data)
    first_derivative = find_first_derivative(data)
    possible_inflexion_points_ind = []
    for i in range(1, len(first_derivative) - 1):
        cur_derivative = first_derivative[i]
        next_derivative = first_derivative[i + 1]
        if cur_derivative > 0 and next_derivative < 0:
            possible_inflexion_points_ind.append(i)
    inflexion_points_ind = []
    for i in range(len(possible_inflexion_points_ind)):
        if second_derivatives[possible_inflexion_points_ind[i] + 1] < -THRESHOLD:
            inflexion_points_ind.append(possible_inflexion_points_ind[i])
    inflexion_points = get_inflexion_points(data, inflexion_points_ind)
    return inflexion_points

def find_first_derivative(data):
    f_derv = []
    for i in range(len(data)):
        cur_angle, cur_dist = data[i].getPolar()
        prev_angle, prev_dist = data[i - 1].getPolar()
        derivative = 0
        if cur_angle == prev_angle:
            if i > 0:
                derivative = f_derv[i - 1]
        #ignore large gaps in data
        elif cur_angle - prev_angle > 5:
            derivative = 0
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
            if i > 0:
                second_derivative = s_derv[i - 1]
        else:
            angle_diff = (cur_angle - prev_angle + 360) % 360
            if angle_diff > 180:
                angle_diff -= 360
            second_derivative = (first_derivatives[i] - first_derivatives[i - 1])/angle_diff
        s_derv.append(second_derivative)
    return s_derv

#window must be odd && drop < window
def create_average_data_set(data, window, drop):
    avg_data_set = []
    half_window = int(window/2)
    for i in range(0, len(data), 10):
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
        angle_total = 0
        for angle in angles:
            angle_total += ((angle - angles[0] + 180) % 360) - 180
        avg_angle = (angles[0] + angle_total/len(angles)) % 360
        avg_dist = np.mean(dists)
        measurement = ptc.Measurement(avg_angle, avg_dist)
        avg_data_set.append(measurement)
    avg_data_set.sort()

    # ptc.configure_scatter_plot()
    # ptc.add_data_scatter(data, 1, "blue")
    # ptc.add_data_scatter(avg_data_set, 1, "red")
    # plt.show()

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

def print_corners(corners, wall_end_points):
    print(f"Location of Points (x, y) [cm]")
    for i in range(len(corners)):
        print(f"{corners[i].getLabel()}: ({corners[i].getCartesian()[0]}, {corners[i].getCartesian()[1]})")
    for i in range(len(wall_end_points)):
        print(f"{wall_end_points[i].getLabel()}: ({wall_end_points[i].getCartesian()[0]}, {wall_end_points[i].getCartesian()[1]})")
    
def display_labels(corners, wall_end_points):
    for i in range(len(corners)):
        plt.text(corners[i].getCartesian()[0] + 5, corners[i].getCartesian()[1], corners[i].getLabel(), fontsize=15, color='black')
    for i in range(len(wall_end_points)):
        plt.text(wall_end_points[i].getCartesian()[0] + 5, wall_end_points[i].getCartesian()[1], wall_end_points[i].getLabel(), fontsize=15, color='black')

def fix_ordering_labels(corners, wall_end_points):
    new_corners = [None] * len(corners)
    for i in range(len(corners)):
        x, y = corners[i].getCartesian()
        if x < 0 and y > 0:
            new_corners[0] = corners[i]
            new_corners[0].setLabel('A')
        elif x > 0 and y > 0:
            new_corners[1] = corners[i]
            new_corners[1].setLabel('B')
        elif x > 0 and y < 0:
            new_corners[2] = corners[i]
            new_corners[2].setLabel('C')
    wall_end_points[0].setLabel('E')
    wall_end_points[1].setLabel('F')
    return new_corners, wall_end_points

def main():
    data = ptc.read_file(DATA_FILE)
    avg_data_set = create_average_data_set(data, 21, 7)
    corners = find_inflexion_points(avg_data_set)
    wall_end_points = find_wall_ends(data)
    corners, wall_end_points = fix_ordering_labels(corners, wall_end_points)
    print_corners(corners, wall_end_points)

    ptc.configure_scatter_plot()
    # ptc.add_data_scatter(avg_data_set, 1, "black")
    ptc.add_data_scatter(data, 1, "blue")
    ptc.add_data_scatter(corners,50, "red")
    ptc.add_data_scatter(wall_end_points, 50, "orange")
    display_labels(corners, wall_end_points)
    plt.show()

    # avg_data_set = create_average_data_set(data, 11, 4)
    # second_derivatives = find_second_derivative(avg_data_set)
    # x_axis = list(range(len(second_derivatives)))
    # plt.scatter(x_axis, second_derivatives, s=1)
    # plt.xlabel("Data Points")
    # plt.ylabel("Second Derivatives")
    # plt.show()

if __name__ == "__main__":
    main()