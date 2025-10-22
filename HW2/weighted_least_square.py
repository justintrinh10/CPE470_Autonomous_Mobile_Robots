import math
import numpy as np
import polar_to_cartesian

DATA_FILES = ["lidar1.txt", "lidar2.txt"]
COVARIANCE = [1, 4]

def weighted_least_square(H, R, y):
    H_transposed = H.T
    rows = R.shape[0]
    R_inverse = R.copy()
    for i in range(rows):
        R_inverse[i][i] = 1/R_inverse[i][i]
    temp1 = np.dot(H_transposed, R_inverse)
    temp1 = np.dot(temp1, H)
    temp1 = np.linalg.inv(temp1)
    temp2 = np.dot(H_transposed, R_inverse)
    temp2 = np.dot(temp2, y)
    return np.dot(temp1, temp2)

def create_covariance_matrix(num1, num2):
    covariance_matrix = np.zeros((num1 + num2, num1 + num2))
    for i in range(num1):
        covariance_matrix[i][i] = COVARIANCE[0]
    for i in range(num1, num1 + num2, 1):
        covariance_matrix[i][i] = COVARIANCE[1]
    return covariance_matrix

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