import numpy as np
from dualquaternion import dual_quaternion_hand_eye_calibration
from dualquaternion.dual_quaternion import *


def assemble_result(b_h, w_e, data_ranges):
    b_h_assembled = []
    w_e_assembled = []
    for data_range in data_ranges:
        begin = data_range[0] - 1
        end = data_range[1]
        for index in range(begin, end):
            b_h_assembled.append(b_h[index])
            w_e_assembled.append(w_e[index])
    return (b_h_assembled, w_e_assembled)

def calculate_delta_transformation_base_in_eye(b_h_assembled, w_e_assembled):
    A = []
    B = []
    for index in range(1, len(b_h_assembled)):
        hand_delta = np.matmul(b_h_assembled[index], np.linalg.inv(b_h_assembled[index -1]))
        eye_delta = np.matmul(w_e_assembled[index], np.linalg.inv(w_e_assembled[index -1]))
        A.append(hand_delta)
        B.append(eye_delta)
    return (A, B)

def calculate_delta_transformation_target_in_hand(b_h_assembled, w_e_assembled):
    A = []
    B = []
    for index in range(1, len(b_h_assembled)):
        A_cur = np.matmul(np.linalg.inv(b_h_assembled[index -1]), b_h_assembled[index])
        B_cur = np.matmul(np.linalg.inv(w_e_assembled[index -1]), w_e_assembled[index])
        A.append(A_cur)
        B.append(B_cur)
    return (A, B)

def format_to_array(np_array):
    result = []
    for i in range(np_array.shape[0]):
        result.append(np_array[i, :, :].transpose())
    return result