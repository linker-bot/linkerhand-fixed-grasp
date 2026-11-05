import numpy as np
from scipy.optimize import minimize, rosen, rosen_der
from scipy.spatial.transform import Rotation

class MinCov:
    def homo_to_v(homo_matrix):
        """
        convert 4x4 matrix into a 1 * 6 vector which contains euler angle and x y z coordinates
        @param homo_matrix: 4 x 4 matrix
        @return: [row yall pitch x y z]
        """
        v = np.zeros((7, 1))
        rotation = Rotation.from_matrix(homo_matrix[0:3, 0:3])
        v[0:3] = rotation.as_euler("xyz", False).reshape(-1, 1)
        #[0:4] = rotation.as_quat().reshape(-1, 1)
        #v[4:7] = homo_matrix[0:3, 3].reshape(-1, 1)
        v[3:6] = homo_matrix[0:3, 3].reshape(-1, 1)
        return v

    def v_to_homo(v):
        """
        convert [row yall pitch x y z] into 4x4 homogenous matriix
        @param v: state vector [row yall pitch x y z]
        @return: 4 x 4 homogenous matrix
        """
        homo = np.zeros((4, 4))
        #rotation = Rotation.from_quat(v[0: 4])
        rotation = Rotation.from_euler( "xyz", v[0: 3], False)
        homo[0:3, 0:3] = rotation.as_matrix()
        homo[0:3, 3] = np.array(v[3:6])
        homo[3, 3] = 1
        return homo

    def calculate_point_to_end(hand_matrix, eye_in_base, point_3d):
        """
        project model points back to end effector based on camera data
        @param hand_matrix: 4 x 4 hand data
        @param eye_in_base: 4 x 4 eye in base matrix
        @param point_3d: 4 x 1 model points in camera frame 3d position
        @return: 3 x 1 model poins projected at end effector  
        """
       
        p1 = np.matmul(np.linalg.inv(hand_matrix), np.linalg.inv(eye_in_base))
        tmp = np.dot(p1, point_3d)
        
        return tmp[0:3]
    
    def calculate_point_to_origin(hand_matrix, eye_in_base, origin_in_end, point_3d):
        """
        project model points back to model origin frame based on camera data
        @param hand_matrix: 4 x 4 hand data
        @param eye_in_base: 4 x 4 eye in base matrix
        @param origin_in_end: 4 x 4 eye origin in end matrix
        @param point_3d: 4 x 1 model points in camera frame 3d position
        @return: 3 x 1 model poins projected at end effector  
        """
        p1 = np.matmul(np.linalg.inv(origin_in_end), np.linalg.inv(hand_matrix))
        p2 = np.matmul(p1, np.linalg.inv(eye_in_base))
       
        tmp = np.dot(p2, point_3d)   
        return tmp[0:3]

    def calculate_vstar(cov_groups):
        """
        calculate cost function based on covariance groups
        @param cov_groups: 4 x 3 x 3 state vector
        @return: cost value
        """   
        eigen_values = np.linalg.eigvals(cov_groups)
        v_prime = np.sum(eigen_values)
        return v_prime

    def calculate_covariance(projected_points):
        """
        calculate covariance of projected points
        @param projected_points: n x 4 x 4 state vector
        @return: 3 x 3 x 4 model poins projected at end effector  
        """
        n_pose = projected_points.shape[0]
        mean_diff = projected_points - np.average(projected_points, axis = 0) # n * 4 * 4
        cov_groups = np.einsum('ijk,ilk->klj',mean_diff,mean_diff) / n_pose
        return cov_groups

    def v_to_v_star_point_to_end(v_iter, hand_data, point_data):
        """
        calculate cost function from state vector for end effector
        @param v_iter: 1x 7 state vector
        @param base_to_cam_iter: 4 x 4 iterative value for base in eye matrix
        @param hand_data: n * 4 * 4
        @param point_data: n * 3 * 4
        @return: 3 x 1 model poins projected at end effector  
        """
        #convert state vector into homogenous matrix 
        base_to_cam_iter = MinCov.v_to_homo(v_iter)
        # make point data n * 4 * 4(homogenous)
        points_n44 = np.ones((point_data.shape[0], 4, point_data.shape[2]))
        points_n44[:, 0:3, :] = point_data
        # calculate projected points
        p1 =  np.einsum('ijk,kl->ijl', np.linalg.inv(hand_data), np.linalg.inv(base_to_cam_iter))
        projected_points = np.einsum('ijk,ikl->ijl',p1, points_n44)
        # calculate covariance
        cov_group = MinCov.calculate_covariance(projected_points)
        # get v
        v_star = MinCov.calculate_vstar(cov_group)
        return v_star 

    
    def v_to_v_star_point_to_origin(v_iter, hand_data, base_in_camera, point_data):
        """
        calculate cost function from state vector for origin
        @param v_iter: 1x 7 state vector
        @param hand_data: n * 4 * 4
        @param point_data: n * 3 * 4
        @return: 3 x 1 model poins projected at end effector  
        """
        #convert state vector into homogenous matrix 
        origin_in_hand_iter = MinCov.v_to_homo(v_iter)
        # make point data n * 4 * 4(homogenous)
        points_n44 = np.ones((point_data.shape[0], 4, point_data.shape[2]))
        points_n44[:, 0:3, :] = point_data
        # calculate projected points    
        p1 =  np.einsum('jk,ikl->ijl', np.linalg.inv(origin_in_hand_iter), np.linalg.inv(hand_data))
        p2 =  np.einsum('ijk,kl->ijl', p1, np.linalg.inv(base_in_camera))
        projected_points = np.einsum('ijk,ikl->ijl',p1, points_n44)
        # calculate covariance
        cov_group = MinCov.calculate_covariance(projected_points)
        # get v
        v_star = MinCov.calculate_vstar(cov_group)
        return v_star 
    
    def get_base_in_eye(hand_data, eye_data, base_to_cam_init, point_data):
        """
        exposed api to calculate base in eye 4x4 matrix
        @param v_iter: 1x 7 state vector
        @param hand_data: n * 4 * 4
        @param base_to_cam_init: 4x4 initial guess
        @param point_data: n * 3 * 4 which contains point data read from camera, currently not being used 
        @return: 4x4 result
        """
        v_init = MinCov.homo_to_v(base_to_cam_init)
        points = np.array([[0.00377, -0.05990, -0.00055, 1],
                            [0.05319, -0.05262, -0.00128, 1],
                        [0.06379, 0.00059, 0.00001, 1],
                        [0.00000, 0.00000, 0.00000, 1]]).transpose()
        point_cloud = np.einsum('ijk,kl->ijl', eye_data, points)[:, 0:3, :]
        res = minimize(fun=MinCov.v_to_v_star_point_to_end, x0=v_init, args=(hand_data, point_cloud), method='Powell',
                                bounds=None, tol=None, callback=None,
                                options={'xtol': 0.0001, 'ftol': 0.0001, 'maxiter': 50000, 'maxfev': None, 'disp': False, 'direc': None, 'return_all': False})


        return MinCov.v_to_homo(res.x)
    
    def get_target_in_hand(hand_data, eye_data, target_in_hand_init, base_to_cam, point_data):
        v_init = MinCov.homo_to_v(target_in_hand_init)
        points = np.array([[0.00377, -0.05990, -0.00055, 1],
                            [0.05319, -0.05262, -0.00128, 1],
                        [0.06379, 0.00059, 0.00001, 1],
                        [0.00000, 0.00000, 0.00000, 1]]).transpose()
        point_cloud = np.einsum('ijk,kl->ijl', eye_data, points)[:, 0:3, :]
        res = minimize(fun=MinCov.v_to_v_star_point_to_origin, x0=v_init, args=(hand_data, base_to_cam, point_cloud), method='Powell',
                            bounds=None, tol=None, callback=None,
                            options={'xtol': 0.0001, 'ftol': 0.0001, 'maxiter': 50000, 'maxfev': None, 'disp': False, 'direc': None, 'return_all': False})
        return MinCov.v_to_homo(res.x)

def main():
    b_h = parse_input_file_from_hand("data_hand.csv")  # ----------(b, b')
    w_e = parse_input_file_from_eye("data_eye.csv")  # -----------(a, a')
    points = np.array([[0.00377, -0.05990, -0.00055],
    [0.05319, -0.05262, -0.00128],
    [0.06379, 0.00059, 0.00001],
    [0.00000, 0.00000, 0.00000]])

    init_guess = np.array([[ 0.02017484, -0.99977372,  0.00674495,  0.02635436]
        ,[-0.47855663, -0.00373325,  0.87804876, -0.19164508]
        ,[-0.87782489, -0.02094234, -0.47852365,  0.79927247]
        , [ 0.,          0.,          0. ,         1.        ]])

    np.set_printoptions(suppress=True)

    res = MinCov.get_base_in_eye(b_h, w_e, init_guess, points)
    
    print(res)

if __name__ == "__main__":
    main()

