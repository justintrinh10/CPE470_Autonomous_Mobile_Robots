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
    print(f"Equation of the line that represents the wall from point {point1} to {point2}:")
    print(f"y = mx + c")
    print(f"m = {x_hat[0][0]}, c = {x_hat[1][0]}")
    print(f"y = {x_hat[0][0]}x + {x_hat[1][0]}")

def display_lines(x_hat, point1, point2, clr):
    line_label = f"Line representing wall {point1} to {point2}"
    plt.axline((0, x_hat[1][0]), slope=x_hat[0][0], color=clr, label=line_label)
     


     
def main():




if __name__ == "__main__":
    main()