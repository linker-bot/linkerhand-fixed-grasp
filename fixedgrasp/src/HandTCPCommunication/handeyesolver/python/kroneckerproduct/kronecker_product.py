import numpy as np
import util as util

class Batch_Processing:
    def pose_estimation(A, B):

        n = A.shape[2];
        T = np.zeros([9, 9]);
        X_est = np.eye(4)
        Y_est = np.eye(4)

        # Permutate A and B to get gross motions
        idx = np.random.permutation(n)
        A = A[:, :, idx];
        B = B[:, :, idx];

        for ii in range(n):

            Ra = A[0:3, 0:3, ii]
            Rb = B[0:3, 0:3, ii]
            #  K[9*ii:9*(ii+1),:] = np.concatenate((np.kron(Rb,Ra), -np.eye(9)),axis=1)
            T = T + np.kron(Rb, Ra);

        U, S, Vt = np.linalg.svd(T)

        xp = Vt.T[:, 0]
        yp = U[:, 0]
        X = np.reshape(xp, (3, 3), order="F")  # F: fortran/matlab reshape order
        Xn = (np.sign(np.linalg.det(X)) / np.abs(np.linalg.det(X)) ** (1 / 3)) * X
        # re-orthogonalize to guarantee that they are indeed rotations.

        U_n, S_n, Vt_n = np.linalg.svd(Xn)
        X = np.matmul(U_n, Vt_n)

        Y = np.reshape(yp, (3, 3), order="F")  # F: fortran/matlab reshape order
        Yn = (np.sign(np.linalg.det(Y)) / np.abs(np.linalg.det(Y)) ** (1 / 3)) * Y
        U_yn, S_yn, Vt_yn = np.linalg.svd(Yn)
        Y = np.matmul(U_yn, Vt_yn)

        A_est = np.zeros([3 * n, 6])
        b_est = np.zeros([3 * n, 1])
        for ii in range(n - 1):
            A_est[3 * ii:3 * ii + 3, :] = np.concatenate((-A[0:3, 0:3, ii], np.eye(3)), axis=1)
            b_est[3 * ii:3 * ii + 3, :] = np.transpose(
                A[0:3, 3, ii] - np.matmul(np.kron(B[0:3, 3, ii].T, np.eye(3)), np.reshape(Y, (9, 1), order="F")).T)

        t_est_np = np.linalg.lstsq(A_est, b_est, rcond=None)
        if t_est_np[2] < A_est.shape[1]:  # A_est.shape[1]=6
            print('Rank deficient')
        t_est = t_est_np[0]
        X_est[0:3, 0:3] = X
        X_est[0:3, 3] = t_est[0:3].T
        Y_est[0:3, 0:3] = Y
        Y_est[0:3, 3] = t_est[3:6].T
        # verify Y_est using rigid_registration
        Y_est_check, ErrorStats = Batch_Processing.__rigid_registration(A, X_est, B)
        return X_est, Y_est, Y_est_check, ErrorStats

    def __rigid_registration(A, X, B):
        # nxnx4
        """solves for Y in YB=AX
        A: (4x4xn)
        B: (4x4xn)
        X= (4x4)
        Y= (4x4)
        n number of measurements
        ErrorStats: Registration error (mean,std)
        """
        n = A.shape[2];
        AX = np.zeros(A.shape)
        AXp = np.zeros(A.shape)
        Bp = np.zeros(B.shape)
        pAX = np.zeros(B[0:3, 3, :].shape)  # To calculate reg error
        pYB = np.zeros(B[0:3, 3, :].shape)  # To calculate reg error
        Y_est = np.eye(4)

        ErrorStats = np.zeros((2, 1))

        for ii in range(n):
            AX[:, :, ii] = np.matmul(A[:, :, ii], X)

            # Centroid of transformations t and that
        t = 1 / n * np.sum(AX[0:3, 3, :], 1);
        that = 1 / n * np.sum(B[0:3, 3, :], 1);
        AXp[0:3, 3, :] = AX[0:3, 3, :] - np.tile(t[:, np.newaxis], (1, n))
        Bp[0:3, 3, :] = B[0:3, 3, :] - np.tile(that[:, np.newaxis], (1, n))

        [i, j, k] = AX.shape;  # 4x4xn
        # Convert AX and B to 2D arrays
        AXp_2D = AXp.reshape((i, j * k))  # now it is 4x(4xn)
        Bp_2D = Bp.reshape((i, j * k))  # 4x(4xn)
        # %Calculates the best rotation
        U, S, Vt = np.linalg.svd(np.matmul(Bp_2D[0:3, :], AXp_2D[0:3, :].T))  # v is v' in matlab
        R_est = np.matmul(Vt.T, U.T)
        # special reflection case
        if np.linalg.det(R_est) < 0:
            print('Warning: Y_est returned a reflection')
            R_est = np.matmul(Vt.T, np.matmul(np.diag([1, 1, -1]), U.T))
            # Calculates the best transformation
        t_est = t - np.dot(R_est, that)
        Y_est[0:3, 0:3] = R_est
        Y_est[0:3, 3] = t_est
        # Calculate registration error
        pYB = np.matmul(R_est, B[0:3, 3, :]) + np.tile(t_est[:, np.newaxis], (1, n))  # 3xn
        pAX = AX[0:3, 3, :]
        Reg_error = np.linalg.norm(pAX - pYB, axis=0)  # 1xn
        ErrorStats[0] = np.mean(Reg_error)
        ErrorStats[1] = np.std(Reg_error)
        return Y_est, ErrorStats


class KronckerProduct:
    def __init__(self, hand_matrix, eye_matrix):
        self.hand_matrix = hand_matrix # A
        self.eye_matrix = eye_matrix # B
        self.sz = len(hand_matrix)
        self.k = 0

        self.rx = None
        self.ry = None
        self.tx = None
        self.ty = None

    def calculate_k(self):
        k = np.zeros([9, 9])
        for i in range(self.sz):
            k += np.kron(self.eye_matrix[i][0:3, 0 :3], self.hand_matrix[i][0:3, 0 :3])
        self.k = k

    def normalize(input):
        u, v, d = np.linalg.svd(input, full_matrices=True)
        return np.matmul(u, d)

    def get_rotation(input_svd):
        det = np.linalg.det(input_svd)
        coff = None
        if det < 0:
            coff = -1 / abs(np.cbrt(det))
        else:
            coff = 1 / abs(np.cbrt(det))
        return coff * input_svd

    def calculate_r(self):
        u, s, vh = np.linalg.svd(self.k, full_matrices= False)

        max_idx = np.argmax(s, axis=0)

        vy = u[:, max_idx].reshape(3, 3, order='F')
        vx = vh.transpose()[:, max_idx].reshape(3, 3, order='F')




        ry = KronckerProduct.get_rotation(vy)
        rx = KronckerProduct.get_rotation(vx)

        self.rx = KronckerProduct.normalize(rx)
        self.ry = KronckerProduct.normalize(ry)


    def calculate_t(self):
        A =  np.block([
                np.eye(3), -self.hand_matrix[0][0:3, 0 :3]
            ])
        b = self.hand_matrix[0][0:3,3].reshape(-1, 1) - np.matmul(self.ry, self.eye_matrix[0][0:3,3].reshape(-1, 1))


        for i in range(1, self.sz):
            temp_A = np.block([
                np.eye(3), -self.hand_matrix[i][0:3, 0:3]
            ])
            temp_b = self.hand_matrix[i][0:3,3].reshape(-1, 1) - np.matmul(self.ry, self.eye_matrix[i][0:3,3].reshape(-1, 1))

            A = np.vstack((A, temp_A))
            b = np.vstack((b, temp_b))

        t = np.linalg.lstsq(A, b, rcond=None)[0]
        self.ty = t[0:3]
        self.tx = t[3:6]


    def get_result(self):

        self.cam_to_base = np.block([
            [self.ry, self.ty.reshape(-1, 1)]
            ,[np.zeros((1,3)), np.ones((1, 1))]
        ])
        self.point_to_end = np.block([
            [self.rx, self.tx.reshape(-1, 1)]
            ,[np.zeros((1, 3)), np.ones((1, 1))]
        ])
        return (self.cam_to_base, self.point_to_end)


def main():
    b_h = util.parse_input_file_from_hand("data_hand.csv")  # ----------(b, b')
    w_e = util.parse_input_file_from_eye("data_eye.csv")  # -----------(a, a')
    b_h_3d = np.ones((4, 4, 23))
    w_e_3d = np.ones((4, 4, 23))

    for i in range(len(b_h)):
        b_h_3d[:, :, i] = b_h[i]
        w_e_3d[:, :, i]  = w_e[i]
    (X_est, Y_est, Y_est_check, ErrorStats) = Batch_Processing.pose_estimation(b_h_3d, w_e_3d)


    kp = KronckerProduct(b_h, w_e)
    kp.calculate_k()
    kp.calculate_r()
    kp.calculate_t()

    result = kp.get_result()
    #print(np.linalg.inv(result[0]))



if __name__ == "__main__":
    main()