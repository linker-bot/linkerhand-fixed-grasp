# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import numpy as np

import posix_ipc
import posix
import os
import errno
import time
from numpy import linalg as LA
import struct
from dq_wrapper import DualQuaternionSolverWrapper
from kronckerproduct_wrapper import KroneckerProductSolverWrapper   
#from mincovariance_wrapper import MinCovarianceSolverWrapper
from iterative.iterative import IterativeSolver
from iterative_eyeinhand.iterative_eyeinhand import IterativeEyeinhandSolver
import util
from scipy.spatial.transform import Rotation as R

semaphore_read = posix_ipc.Semaphore("/send_row_data_semaphore", posix_ipc.O_CREAT)

def add_single_noise_to_se4(original_data, mu_rotation, sigma_rotation, mu_translation, sigma_translation):
    rotation = original_data[0:3, 0:3]
    translation = original_data[0:3, 3:4]
    # add noise to rotation
    r = R.fromMatrix(rotation)
    q = r.as_quat()
    r_noise = R.from_rotvec(np.random.normal(mu_rotation, sigma_rotation, 3))
    q_noise = r_noise.as_quat()
    q_final = q_noise * q
    rotation_final = q_final.as_matrix()
    # add noise to translation
    t_noise = np.random.normal((mu_translation, sigma_translation, 3))
    translation_final = translation + t_noise
    # combine rotation and translation to result
    result = np.zeros((4, 4))
    result[0:3, 0:3] = rotation_final
    result[0:3, 3:4] = translation_final
    result[3, 3] = 1
    return result


def add_noise_to_data(hand_data, eye_data):
    hand_data_new = np.zeros(hand_data.shape)
    eye_data_new = np.zeros(eye_data.shape)
    for i in range(hand_data.shape[0]):
        hand_data_new[i, :, :] = add_single_noise_to_se4(hand_data[i, :, :], 0, 0.1, 0, 0.1)
        eye_data_new[i, :, :] = add_single_noise_to_se4(eye_data_new[i, :, :], 0, 0.1, 0, 0.1)
    return (hand_data_new, eye_data_new)

def main():
    try:
        os.mkfifo("./handeyeoriginalpath")
    except OSError as oe: 
        if oe.errno != errno.EEXIST:
            raise
   
    read_fifo = posix.open("./handeyerawdata", posix.O_RDONLY) 
    write_fifo = posix.open("./handeyeresult", posix.O_WRONLY) 
    print("python connection set up")
    method_count = int.from_bytes(posix.read(read_fifo, 1), byteorder = "little", signed = False) 
    methods = []
    for i in range(method_count):
        methods.append(int.from_bytes(posix.read(read_fifo, 1), byteorder = "little", signed = False) )
    #data_size = int(struct.unpack('d', posix.read(read_fifo, 8))[0])
    data_size =  int.from_bytes(posix.read(read_fifo, 1), byteorder = "little", signed = False)
    print("data size is ", data_size)
    data = posix.read(read_fifo, 2 * 16 * 8 * data_size) 
    data_range_cand = [[1, data_size]]
    if(len(data) > 0):
        
        dt = np.dtype(np.double)
        dt = dt.newbyteorder('<')
        input = np.frombuffer(data, dtype=dt).reshape(data_size * 2, 4, 4)
        hand_data = util.format_to_array(input[0 : data_size, :, :])
        eye_data = util.format_to_array(input[data_size : data_size * 2, :, :])
        (hand_data_selected, eye_data_selected) = util.assemble_result(hand_data, eye_data, data_range_cand)
        #hand_data_selected, eye_data_selected = add_noise_to_data(hand_data_selected, eye_data_selected)
        #print("python:", hand_data_selected)
        #print("python", eye_data_selected)
        result = []
        for i in range(method_count):
            try:    
                if methods[i]== 1:
                    result.append(DualQuaternionSolverWrapper.calculate_result(hand_data_selected,eye_data_selected))
                elif methods[i] == 2:
                    result.append(KroneckerProductSolverWrapper.calculate_result(hand_data_selected,eye_data_selected))
                elif methods[i] == 3:
                    result.append(IterativeEyeinhandSolver.calculate_result(hand_data_selected,eye_data_selected))
                    print("methods: ", i, "IterativeEyeinhandSolver")
                elif methods[i] == 4:
                    result.append(IterativeSolver.calculate_result(hand_data_selected, eye_data_selected))
                    print("methods: ", i, "IterativeSolver")
            except:
                print("methods: ", i, " has erros:, data mismatch")
                result.append(np.zeros((2, 4, 4)))
        result_bytes = np.array(result).reshape(method_count * 2, 4, 4)
        #print(result_bytes)
        posix.writev(write_fifo, result_bytes)

    time.sleep(0.1) 
    posix.close(write_fifo)
    posix.close(read_fifo)
    print("calculation end")

if __name__ == "__main__":
    main()


