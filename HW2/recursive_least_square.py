import math
import numpy as np
import polar_to_cartesian
import weighted_least_square

DATA_FILE = "streamdata.txt"
COVARIANCE = [1, 4]

def calculate_recursive_least_square(x_hat_prev, gain_matrix, residual):
    change = np.dot(gain_matrix, residual)
    return x_hat_prev + change

def calculate_gain_matrix(prev_estimator_covariance, jacobian_matrix, measurement_covariance):
    H_transposed = jacobian_matrix.T
    temp1 = np.dot(prev_estimator_covariance, H_transposed)
    temp2 = np.dot(jacobian_matrix, prev_estimator_covariance)
    temp2 = np.dot(temp2, H_transposed)
    temp2 = temp2 + measurement_covariance
    temp2 = np.linalg.inv(temp2)
    return np.dot(temp1, temp2)

def calculate_residual(y_vector, jacobian_matrix, prev_recursive_least_square):
    temp = np.dot(jacobian_matrix, prev_recursive_least_square)
    return y_vector - temp

def calculate_estimator_covariance(gain_matrix, jacobian_matrix, prev_estimator_covariance):
    I = np.eye(prev_estimator_covariance.shape[0])
    temp = I - np.dot(gain_matrix, jacobian_matrix)
    return np.dot(temp, prev_estimator_covariance)

def read_file(file_name):
    measurements = []
    with open(file_name, 'r') as file:
        file.readline()
        for line in file:
            line = line.strip()
            if not line:
                continue
            items = line.split(',')
            lidar = int(items[0])
            angle = float(items[1])
            dist = float(items[2])
            measurements.append(polar_to_cartesian.Measurement(angle, dist, lidar))
    return measurements

def create_measurement_covariance_matrix(measurement_covariances):
    measurement_covariance_matrix = np.zeros((len(measurement_covariances), len(measurement_covariances)))
    for i in range(len(measurement_covariances)):
        measurement_covariance_matrix[i][i] = measurement_covariances[i]
    return measurement_covariance_matrix

def create_jacobian_matrix(x_i):
    return np.array([[x_i, 1]])

def create_y_vector(y_i):
    return np.array([[y_i]])


def output(X_hat, estimator_covariance):
    print(f"Equation of the line that represents the wall:")
    print(f"y = mx + c")
    print(f"m = {X_hat[0][0]}, c = {X_hat[1][0]}")
    print(f"y = {X_hat[0][0]}x + {X_hat[1][0]}")
    print(f"Estimator Covariance")
    print(f"P = [{estimator_covariance[0][0]}, 0]")
    print(f"    [0, {estimator_covariance[1][1]}]")
    print()

def main():
    initial_m, initial_c = weighted_least_square.main()
    print()

    estimator_covariance_matrix = np.zeros((2, 2))
    initial_covariance_m = math.tan(math.radians(5)) ** 2
    initial_covariance_c = 1
    estimator_covariance_matrix[0][0] = initial_covariance_m
    estimator_covariance_matrix[1][1] = initial_covariance_c
    recursive_least_square_matrix = np.empty((2, 1))
    recursive_least_square_matrix[0][0] = initial_m
    recursive_least_square_matrix[1][0] = initial_c
    
    print(f"Initial Estimate:")
    output(recursive_least_square_matrix, estimator_covariance_matrix)

    data_stream = read_file(DATA_FILE)

    i = 1
    for measurement in data_stream:
        x_cord, y_cord = measurement.getCartesian()
        H = create_jacobian_matrix(x_cord)
        y_vector = create_y_vector(y_cord)

        if measurement.getLidar() == 1:
            R = np.array([[1]])
        else:
            R = np.array([[4]])

        K = calculate_gain_matrix(estimator_covariance_matrix, H, R)
        residual = calculate_residual(y_vector, H, recursive_least_square_matrix)
        recursive_least_square_matrix = calculate_recursive_least_square(recursive_least_square_matrix, K, residual)
        estimator_covariance_matrix = calculate_estimator_covariance(K, H, estimator_covariance_matrix)

        print(f"Number {i} Estimate")
        output(recursive_least_square_matrix, estimator_covariance_matrix)
        i += 1


if __name__ == "__main__":
    main()