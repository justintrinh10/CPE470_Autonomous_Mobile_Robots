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
    print()

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

def create_and_display_line(data, point1, point2, clr):
    subset = create_subset(data, point1, point2)
    subset_x_hat = least_square(subset)
    print_equation_line(subset_x_hat, point1, point2)
    display_lines(subset_x_hat, point1, point2, clr)
    return subset_x_hat

def find_intersection(x_hat1, x_hat2):
    m1 = x_hat1[0][0]
    m2 = x_hat2[0][0]
    b1 = x_hat1[1][0]
    b2 = x_hat2[1][0]
    x = (b2 - b1)/(m1 - m2)
    y = m1 * x + b1
    angle, dist = cartesian_to_polar(x, y)
    intersection = ptc.Measurement(angle, dist)
    intersection.setLabel("D")
    return intersection

def cartesian_to_polar(x, y):
    dist = math.sqrt(x*x + y*y)
    angle = math.degrees(math.atan2(y, x))
    return angle, dist

def find_wall_opening(corners, wall_end_points):
    wall_opening = []
    wall_opening.append(corners[3])
    wall_opening.append(wall_end_points[0])
    return wall_opening

def calculate_distance(point1, point2):
    x1, y1 = point1.getCartesian()
    x2, y2 = point2.getCartesian()
    x_diff = x2 - x1
    y_diff = y2 - y1
    return math.sqrt(x_diff*x_diff + y_diff*y_diff)

def main():
    data = ptc.read_file(DATA_FILE)
    avg_data_set = ip.create_average_data_set(data, 21, 7)
    corners = ip.find_inflexion_points(avg_data_set)
    wall_end_points = ip.find_wall_ends(data)
    corners, wall_end_points = ip.fix_ordering_labels(corners, wall_end_points)

    ptc.configure_scatter_plot()
    ptc.add_data_scatter(data, 1, "blue")

    #Wall A to E
    a_e_line = create_and_display_line(data, corners[0], wall_end_points[0], "orange")
    #Wall F to C
    f_c_line = create_and_display_line(data, wall_end_points[1], corners[2], "yellow")
    #Wall C to B
    create_and_display_line(data, corners[2], corners[1], "green")
    #Wall B to A
    create_and_display_line(data, corners[1], corners[0], "purple")

    intersection = find_intersection(a_e_line, f_c_line)
    corners.append(intersection)

    ip.print_corners(corners, wall_end_points)
    print()
    ptc.add_data_scatter(corners,50, "red")
    ptc.add_data_scatter(wall_end_points, 50, "red")
    ip.display_labels(corners, wall_end_points)
    print()
    
    wall_opening = find_wall_opening(corners, wall_end_points)
    print("Location of Wall Opening")
    for i in range(len(wall_opening)):
        print(f"{wall_opening[i].getLabel()}: ({wall_opening[i].getCartesian()[0]}, {wall_opening[i].getCartesian()[1]})")
    width = calculate_distance(wall_opening[0], wall_opening[1])
    print(f"Width of Opening: {width} cm")

    plt.xlim((-150, 150))
    plt.ylim((-150, 150))
    plt.show()

if __name__ == "__main__":
    main()