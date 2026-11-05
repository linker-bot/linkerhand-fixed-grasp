import math
import numpy as np
from pyquaternion import Quaternion

class IterativeSolver:

    def calculate_result(hand_data, eye_data):
        y_pose = IterativeSolver.AXXB(hand_data, eye_data)
        hand_data_inverse = np.linalg.inv(np.array(hand_data))
        eye_data_inverse = np.linalg.inv(np.array(eye_data))
        hand_data_inverse_list = hand_data_inverse.tolist()
        eye_data_inverse_list = eye_data_inverse.tolist()
        x_pose = IterativeSolver.AXXB(hand_data_inverse_list, eye_data_inverse_list)
        return (np.linalg.inv(x_pose), y_pose)

    def AXXB(AXXB_fk_list,AXXB_ndi_list):
        bias = 0.5

        nums = len(AXXB_fk_list)

        if nums >= 4:
            # calibration
            A = np.array([
                np.mat(AXXB_fk_list[i + 1]).I @ np.mat(AXXB_fk_list[i]) for i in range(nums - 1)
            ])
            B = np.array([
                np.mat(AXXB_ndi_list[i + 1]).I @ np.mat(AXXB_ndi_list[i]) for i in range(nums - 1)
            ])

            rotationA = np.array([A[i, 0:3, 0:3] for i in range(nums - 1)])
            rotationB = np.array([B[i, 0:3, 0:3] for i in range(nums - 1)])
            translationA = np.array([A[i, 0:3, 3] for i in range(nums - 1)])
            translationB = np.array([B[i, 0:3, 3] for i in range(nums - 1)])

            Rx, tx = IterativeSolver.find_rotation_and_translation(rotationA[0:nums - 2], rotationB[0:nums - 2], translationA[0:nums - 2], translationB[0:nums - 2])
            X = np.eye(4)
            X[0:3, 0:3] = Rx
            X[0:3, 3] = tx.reshape(1, 3)
            marker_flange_data = X  # marker_flange

            # filter,loop 3 times
            loopA = rotationA
            loopB = rotationB
            ltA = translationA
            ltB = translationB
            for i in range(0):
                rotA = loopA
                rotB = loopB
                tA = ltA
                tB = ltB
                loopA = []
                loopB = []
                ltB = []
                ltA = []
                E1 = []
                E2 = []

                rotA = np.array(rotA)
                times = rotA.shape[0]
                for j in range(times):
                    dis = IterativeSolver.ErrorEst(marker_flange_data, rotA[j], rotB[j], tA[j], tB[j])
                    E1.append(dis)
                E1 = np.array(E1)
                m1 = np.median(E1)
                for k in range(times):
                    dis2 = abs(m1 - E1[k])
                    E2.append(dis2)
                E2 = np.array(E2)
                m2 = np.median(E2)
                print(len(E2), '*' * 50, m2)
                for j in range(times):
                    if E2[j] <= 2.0 * m2:
                        loopA.append(rotA[j])
                        loopB.append(rotB[j])
                        ltA.append(tA[j])
                        ltB.append(tB[j])

                loopA = np.array(loopA)
                loopB = np.array(loopB)
                ltA = np.array(ltA)
                ltB = np.array(ltB)
                size = loopA.shape[0]
                print(size)
                Rx, tx =IterativeSolver.find_rotation_and_translation(loopA[0:size - 2], loopB[0:size - 2], ltA[0:size - 2], ltB[0:size - 2])

                X = np.eye(4)
                X[0:3, 0:3] = Rx
                X[0:3, 3] = tx.reshape(1, 3)

                marker_flange_data = X

        return marker_flange_data


    def find_rotation_and_translation(RA, RB, translationA, translationB):
        '''计算手眼标定齐次矩阵'''
        V1 = []
        V2 = []
        for i in range(RA.shape[0]):
            V1i = IterativeSolver.findrotationaxis(RA[i])
            V1.append(V1i)
            V2i = IterativeSolver.findrotationaxis(RB[i])
            V2.append(V2i)
        V1 = np.array(V1)
        V2 = np.array(V2)
        A = np.zeros((4, 4))
        for i in range(V1.shape[0]):
            Ci = IterativeSolver.Q(V1[i, :]) - IterativeSolver.W(V2[i, :])
            Ai = np.dot(Ci.T, Ci)
            A = A + Ai
        eig = np.linalg.eig(A)
        eigenvalue = eig[0]
        eigenvector = eig[1]
        alpha = 10 ** 5
        for j in range(eigenvalue.size):
            if -0.00001 < eigenvalue[j] < alpha:
                alpha = eigenvalue[j]
        delta = eigenvalue.tolist().index(alpha)
        qx = Quaternion(array=eigenvector[:, delta])
        Rx = qx.rotation_matrix

        R = []
        C = []
        for k in range(translationA.shape[0]):
            Rk = RA[k] - np.eye(3, dtype=int)
            R.append(Rk)
            Ck = (np.dot(Rx, translationB[k, :]) - translationA[k, :]).reshape(3, 1)
            C.append(Ck)
        r = np.array(R).reshape(RA.shape[0] * 3, 3)
        c = np.array(C).reshape(RA.shape[0] * 3, 1)
        tx = np.dot(np.linalg.inv(r.T @ r), np.dot(r.T, c))
        return Rx, tx


    def ErrorEst(marker_flange_data, A, B, ta, tb):
        tx = marker_flange_data[0:3,3].reshape(3,1)
        rx = marker_flange_data[0:3,0:3]
        D = A @ tx + ta.reshape(3,1) - rx @ tb.reshape(3,1) - tx
        dis = np.linalg.norm(D)
        return abs(dis)


    def findrotationaxis(R):
        '''旋转轴计算'''
        theta = math.acos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
        V = np.array([[R[2, 1] - R[1, 2]], [R[0, 2] - R[2, 0]], [R[1, 0] - R[0, 1]]]) / (2 * math.sin(theta))
        return V


    def Q(v):
        '''用向量生成特定矩阵'''
        u0 = np.zeros((4,1))
        u0[0] = 0
        u0[1] = v[0]
        u0[2] = v[1]
        u0[3] = v[2]
        u1 = np.zeros((4,1))
        u1[0] = -v[0]
        u1[1] = 0
        u1[2] = v[2]
        u1[3] = -v[1]
        u2 = np.zeros((4,1))
        u2[0] = -v[1]
        u2[1] = -v[2]
        u2[2] = 0
        u2[3] = v[0]
        u3 = np.zeros((4,1))
        u3[0] = -v[2]
        u3[1] = v[1]
        u3[2] = -v[0]
        u3[3] = 0
        Q = np.c_[u0,u1,u2,u3]
        return Q


    def W(v):
        '''用向量生成特定矩阵'''
        u0 = np.zeros((4,1))
        u0[0] = 0
        u0[1] = v[0]
        u0[2] = v[1]
        u0[3] = v[2]
        u1 = np.zeros((4,1))
        u1[0] = -v[0]
        u1[1] = 0
        u1[2] = -v[2]
        u1[3] = v[1]
        u2 = np.zeros((4,1))
        u2[0] = -v[1]
        u2[1] = v[2]
        u2[2] = 0
        u2[3] = -v[0]
        u3 = np.zeros((4,1))
        u3[0] = -v[2]
        u3[1] = -v[1]
        u3[2] = v[0]
        u3[3] = 0
        W = np.c_[u0,u1,u2,u3]
        return W

if __name__ == "__main__":
    fk_list = [
    [[-0.0846,    0.1486 ,   0.9853 ,   0.7324],[-0.0818 ,   0.9844 ,  -0.1555 ,  -0.1992],[-0.9931  , -0.0937 ,  -0.0711 ,   0.4122],[0,0,0,1]],
    [[0.0406  ,  0.1674  ,  0.9851  ,  0.6016],[-0.1490  ,  0.9759  , -0.1596  , -0.3535],[-0.9880  , -0.1403  ,  0.0646  ,  0.4095],[0,0,0,1]],
    [[-0.8751  ,  0.3456 ,   0.3386 ,   0.6584],[ 0.3319 ,   0.9380 ,  -0.0996 ,  -0.1016],[ -0.3521,    0.0252,   -0.9356,    0.3626],[0,0,0,1]],
    [[-0.1629  ,  0.1603 ,   0.9735 ,   0.8315],[-0.2448  ,  0.9493 ,  -0.1973 ,  -0.1022],[-0.9558 ,  -0.2704 ,  -0.1154 ,   0.4913],[0,0,0,1]],
    [[-0.6778  ,  0.1245 ,   0.7246 ,   0.5782],[0.0181  ,  0.9881  , -0.1529  , -0.2097],[-0.7350  , -0.0905  , -0.6720  ,  0.6870],[0,0,0,1]]]
    ndi_list = [
    [[0.3401  ,  0.5575  ,  0.7573  ,  0.0209],[ 0.0624  , -0.8169  ,  0.5734  ,  0.1057],[0.9383  , -0.1477   ,-0.3126 ,  -1.5097],[0,0,0,1]],
    [[0.2964  ,  0.6596  ,  0.6907  ,  0.0850],[0.0427  , -0.7316   , 0.6804   , 0.0058],[0.9541   ,-0.1721   ,-0.2450  , -1.3431],[0,0,0,1]],
    [[0.4422  , -0.5055  ,  0.7409  ,  0.1231],[ -0.1390 ,  -0.8547 ,  -0.5002 ,  -0.0319],[0.8861 ,   0.1182 ,  -0.4482,   -1.6320],[0,0,0,1]],
    [[ 0.1610 ,   0.5392 ,   0.8267 ,  -0.0986],[ 0.0484 ,  -0.8409 ,   0.5390 ,   0.1843],[ 0.9858,   -0.0468 ,  -0.1615,   -1.5847],[0,0,0,1]],
    [[ 0.3408 ,  -0.0208 ,   0.9399 ,  -0.1565],[0.0908  , -0.9944  , -0.0549  , -0.0472],[0.9358  ,  0.1040   ,-0.3370  , -1.3769],[0,0,0,1]]]
    print(len(ndi_list))
    maker_flange_dat = IterativeSolver.AXXB(fk_list, ndi_list)
    print(maker_flange_dat)
    FK = np.array([[ -0.1321 ,   0.1839   , 0.9740 ,   0.4877],[ -0.2530   , 0.9438 ,  -0.2125  , -0.4681],[ -0.9584   ,-0.2745 ,  -0.0781 ,   0.5257],[0,0,0,1]])
    NDI = np.array([[0.1626 ,   0.5648   , 0.8091  ,  0.0450],[0.0247  , -0.8221    ,0.5689  , -0.0764],[ 0.9864  , -0.0725 ,  -0.1476 ,  -1.1744],[0,0,0,1]])
    T_base_ndi_v= FK @ maker_flange_dat @ np.linalg.inv(NDI)
    T_base_ndi_o= np.array(fk_list[2]) @ maker_flange_dat @ np.linalg.inv(np.array(ndi_list[2]))
    print(T_base_ndi_v)
    print(T_base_ndi_o)
