import numpy as np
from dualquaternion import dual_quaternion_hand_eye_calibration
from dualquaternion.dual_quaternion import *

import util as util



class DualQuaternionSolverWrapper:
    def convert_to_dual_quat(hand_data_homo, eye_data_homo):
        hand_data_dq = []
        eye_data_dq = []
        for index in range(0, len(hand_data_homo)):
            hand_data_dq.append(DualQuaternion.from_transformation_matrix(hand_data_homo[index]))
            eye_data_dq.append(DualQuaternion.from_transformation_matrix(eye_data_homo[index]))
        return (hand_data_dq, eye_data_dq)
    
    def add_origin(dual_hand_data, dual_eye_data):
        origin = np.array([[ 1, 0, 0, 0],
                [0, 1, 0, 0],
                [ 0.000000, 0.000000, 1, 0],
                [ 0, 0, 0, 1]])
        dual_origin = DualQuaternion.from_transformation_matrix(origin)
        hand_data = [dual_origin]
        hand_data.extend(dual_hand_data)
        eye_data = [dual_origin]
        eye_data.extend(dual_eye_data)
        return (hand_data, eye_data)
    
    def apply_ransac_calibration(hand_data_dq, eye_data_dq):
        hand_eye_config = dual_quaternion_hand_eye_calibration.HandEyeConfig()
        hand_eye_config.visualize = False
        hand_eye_config.ransac_max_number_iterations = 50
        hand_eye_config.ransac_sample_size = 3
        (success, dq_H_E_estimated, rmse,
        num_inliers, num_poses_kept,
        runtime, singular_values, bad_singular_values) = \
        dual_quaternion_hand_eye_calibration.compute_hand_eye_calibration_RANSAC(hand_data_dq, eye_data_dq,
                                                                hand_eye_config)
        return dq_H_E_estimated.to_matrix()

    def calculate_base_eye(hand_data_input, eye_data_input):
        """
        Solving AX = XB while A = H2(hand)^-1 * H1(hand), B = H2(eye)^-1 * H1(eye)
        @param hand_data_input: List contains 4x4 end effector transformation matrix 
        @param eye_data_input : List contains 4x4 target in camera frame transformation matrix 
        """
        (hand_data_delta, eye_data_delta) = util.calculate_delta_transformation_base_in_eye(hand_data_input, eye_data_input)
        (hand_data_dq, eye_data_dq) = DualQuaternionSolverWrapper.convert_to_dual_quat(hand_data_delta, eye_data_delta)
        (hand_data_dq, eye_data_dq) = DualQuaternionSolverWrapper.add_origin(hand_data_dq, eye_data_dq)
        
        #result = dual_quaternion_hand_eye_calibration.compute_hand_eye_calibration(hand_data_dq, eye_data_dq)[0].to_matrix()
        result = DualQuaternionSolverWrapper.apply_ransac_calibration(hand_data_dq, eye_data_dq)
        return result
    
    def calculate_target_hand(hand_data_input, eye_data_input):
        """
        Solving AX = XB while A = H2(hand)^-1 * H1(hand), B = H2(eye)^-1 * H1(eye)
        @param hand_data_input: List contains 4x4 end effector transformation matrix 
        @param eye_data_input : List contains 4x4 target in camera frame transformation matrix 
        """
        (hand_data_delta, eye_data_delta) = util.calculate_delta_transformation_target_in_hand(hand_data_input, eye_data_input)
        (hand_data_dq, eye_data_dq) = DualQuaternionSolverWrapper.convert_to_dual_quat(hand_data_delta, eye_data_delta)
        (hand_data_dq, eye_data_dq) = DualQuaternionSolverWrapper.add_origin(hand_data_dq, eye_data_dq)
        #result = dual_quaternion_hand_eye_calibration.compute_hand_eye_calibration(hand_data_dq, eye_data_dq)[0].to_matrix # the first input is B, the second input is A
        result = DualQuaternionSolverWrapper.apply_ransac_calibration(hand_data_dq, eye_data_dq)
        return np.linalg.inv(result)
        
    def calculate_result(hand_data_input, eye_data_input):
        base_to_eye = DualQuaternionSolverWrapper.calculate_base_eye(hand_data_input, eye_data_input)
        target_to_hand = DualQuaternionSolverWrapper.calculate_target_hand(hand_data_input, eye_data_input)
        
        return (base_to_eye, target_to_hand)
    
