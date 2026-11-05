import numpy as np
from mincovariance.min_covariance import MinCov
from kronckerproduct_wrapper import KroneckerProductSolverWrapper
class MinCovarianceSolverWrapper:
     def calculate_result(hand_data, eye_data):
            init_guess = KroneckerProductSolverWrapper.calculate_result(hand_data,eye_data)
            points = np.array([[0.00377, -0.05990, -0.00055],
                    [0.05319, -0.05262, -0.00128],
                    [0.06379, 0.00059, 0.00001],
                    [0.00000, 0.00000, 0.00000]])
            hand_data = np.array(hand_data)
            eye_data = np.array(eye_data)
            base_to_camera = MinCov.get_base_in_eye(hand_data, eye_data, init_guess[0], points)
            origin_to_end = MinCov.get_target_in_hand(hand_data, eye_data, init_guess[1], base_to_camera, points)
            return (base_to_camera, origin_to_end)
          