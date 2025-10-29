import math
import matplotlib.pyplot as plt
import numpy as np
import polar_to_cartesian as ptc
import inflexion_points as ip

DATA_FILE = "lidar_data.csv"

def least_square(data):
    H_matrix = create_jacobian_matrix(data)
    y_vec = create_y_vector(data)
    X_hat = least_square_equation(H_matrix, y_vec)
    return X_hat

def least_square_equation(H, y):
    H_transposed = H.T
    temp1 = np.dot(H_transposed, H)
    temp1 = np.linalg.inv(temp1)
    temp2 = np.dot(H_transposed, y)
    return np.dot(temp1, temp2)

def create_jacobian_matrix(data):
    jacobian_matrix = np.empty((len(data), 2))
    for i in range(len(data)):
            x, y = data[i].getCartesian()
            jacobian_matrix[i][0] = x
            jacobian_matrix[i][1] = 1
    return jacobian_matrix

def create_y_vector(data):
    y_vector = np.empty((len(data), 1))
    for i in range(len(data)):
        x, y = data[i].getCartesian()
        y_vector[i][0] = y
    return y_vector

def print_equation_line(x_hat, point1, point2):
    print(f"Equation of the line that represents the wall from point {point1.getLabel()} to {point2.getLabel()}:")
    print(f"y = mx + c")
    print(f"m = {x_hat[0][0]}, c = {x_hat[1][0]}")
    print(f"y = {x_hat[0][0]}x + {x_hat[1][0]}")

def display_lines(x_hat, point1, point2, clr):
    line_label = f"Line representing wall {point1.getLabel()} to {point2.getLabel()}"
    plt.axline((0, x_hat[1][0]), slope=x_hat[0][0], color=clr, label=line_label)
     
def create_subset(data, point1, point2):
    data_subset = []
    start_angle = point1.getPolar()[0]
    end_angle = point2.getPolar()[0]
    if start_angle <= end_angle:
        for i in range(len(data)):
            cur_angle = data[i].getPolar()[0]
            if start_angle <= cur_angle and cur_angle <= end_angle:
                data_subset.append(data[i])
    else:
        for i in range(len(data)):
            cur_angle = data[i].getPolar()[0]
            if cur_angle >= start_angle or cur_angle <= end_angle:
                data_subset.append(data[i])
    return data_subset

def main():
    data = ptc.read_file(DATA_FILE)
    avg_data_set = ip.create_average_data_set(data, 21, 7)
    corners = ip.find_inflexion_points(avg_data_set)
    wall_end_points = ip.find_wall_ends(data)
    corners, wall_end_points = ip.fix_ordering_labels(corners, wall_end_points)





if __name__ == "__main__":
    main()