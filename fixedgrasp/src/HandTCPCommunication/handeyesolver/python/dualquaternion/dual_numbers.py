import numpy as np
from numbers import Number

class DualNumber(object):
    def __init__(self, d_real = 0, d_dual = 0):
        self.dn = [d_real, d_dual]

    def __str__(self):
        return "[real: {}, dual: {}]".format(str(self.dn[0]), str(self.dn[1]))

    def __add__(self, other):
        """ Dual Number addition. """
        d_real= self.dn[0] + other.dn[0]
        d_dual = self.dn[1] + other.dn[1]
        return DualNumber(d_real, d_dual)

    def __sub__(self, other):
        """ Dual Number substration. """
        d_real = self.dn[0] - other.dn[0]
        d_dual = self.dn[1] - other.dn[1]
        return DualNumber(d_real, d_dual)

    def __mul__(self, other):
        """ Dual Number multiply """
        if isinstance(other, DualNumber):
            d_real = self.dn[0] * other.dn[0]
            d_dual = self.dn[0] * other.dn[1] + self.dn[1] * other.dn[0]
            return DualNumber(d_real, d_dual)
        elif(isinstance(other, Number)):
            d_real = self.dn[0] * other
            d_dual = self.dn[1] * other
            return DualNumber(d_real, d_dual)

class DualMatrix(object):
    def __init__(self, dual_matrix):
        self.dual_matrix = dual_matrix

    def __add__(self, other):
        """ Dual Matrix addition. """
        dual_matrix = self.dual_matrix + other.dual_matrix
        return DualMatrix(dual_matrix)

    def __sub__(self, other):
        """ Dual Matrix substration. """
        dual_matrix = self.dual_matrix + other.dual_matrix
        return DualMatrix(dual_matrix)

class HandEyeSolver(object):
    def __init__(self, hand_data_se3, eye_data_se3):
        self.hand_data_se3 = hand_data_se3
        self.eye_data_se3 = eye_data_se3

def main():
    d1 = DualNumber(1, 2)
    d2 = DualNumber(3, 4)
    m1 = np.array([[d1, d2], [d2, d1]])
    m2 = np.array([[d2, d1], [d1, d2]])
    m3 = np.matmul(m1, m2)

if(__name__ != "main"):
    main()