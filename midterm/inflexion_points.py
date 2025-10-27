import polar_to_cartesian as ptc
import math
import numpy
import matplotlib as plt

def find_inflexion_points(data):

def find_first_derivative(data):
    f_derv = []
    for i in range(len(data)):
        cur_angle, cur_dist = data[i].getPolar()
        prev_angle, prev_dist = data[i - 1].getPolar()
        derivative = (cur_dist - prev_dist)/(cur_angle - prev_angle)
        f_derv.append(derivative)
    return f_derv

def find_second_derivative(data):
    first_derivatives = find_first_derivative(data)
    s_derv = []
    for i in range(len(first_derivatives)):
        cur_angle, trash = data[i].getPolar()
        prev_angle, trash = data[i - 1].getPolar()
        second_derivative = (first_derivatives[i] - first_derivatives[i - 1])/(cur_angle - prev_angle)
        s_derv.append(second_derivative)
    return s_derv


def main():


if __name__ == "__main__":
    main()