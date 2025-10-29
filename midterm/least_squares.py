import math
import matplotlib.pyplot as plt
import numpy as np
import polar_to_cartesian as ptc
import inflexion_points as ip

DATA_FILE = "lidar_data.csv"

def least_square(H, y):
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



def main():
    dataSet1 = polar_to_cartesian.read_file(DATA_FILES[0], 0)
    dataSet2 = polar_to_cartesian.read_file(DATA_FILES[1], 1)
    completeDataSet = dataSet1 + dataSet2
    H_matrix = create_jacobian_matrix(completeDataSet)
    y_vec = create_y_vector(completeDataSet)
    covar_matrix = create_covariance_matrix(len(dataSet1), len(dataSet2))
    X_hat = weighted_least_square(H_matrix, covar_matrix, y_vec)
    print(f"Weight Least Squares")
    print(f"Equation of the line that represents the wall:")
    print(f"y = mx + c")
    print(f"m = {X_hat[0][0]}, c = {X_hat[1][0]}")
    print(f"y = {X_hat[0][0]}x + {X_hat[1][0]}")
    return X_hat[0][0], X_hat[1][0]

if __name__ == "__main__":
    main()