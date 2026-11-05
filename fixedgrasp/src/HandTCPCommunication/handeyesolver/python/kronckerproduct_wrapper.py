import numpy as np
from kroneckerproduct.kronecker_product import KronckerProduct


class KroneckerProductSolverWrapper:
      def calculate_result(hand_data, eye_data):
            kp = KronckerProduct(hand_data, eye_data)
            kp.calculate_k()
            kp.calculate_r()
            kp.calculate_t()
            result = kp.get_result()
            return (np.linalg.inv(result[0]), result[1])
