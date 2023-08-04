import numpy as np

from rotations import Quaternion, omega, skew_symmetric, angle_normalize
import weakref

class ExtendedKalmanFilter:
    def __init__(self, gnss_obj, imu_obj, vehicle_obj):
        self.gnss_obj = gnss_obj
        self.imu_obj = imu_obj
        self.vehicle_obj = vehicle_obj

        # State (position, velocity and orientation)
        self.p = np.zeros([3, 1])
        self.v = np.zeros([3, 1])
        self.q = np.zeros([4, 1])  # quaternion

        # State covariance
        self.p_cov = np.zeros([9, 9])
        # Low uncertainty in position estimation and high in orientation and velocity
        pos_var = 1
        orien_var = 1000
        vel_var = 1000
        self.p_cov[:3, :3] = np.eye(3) * pos_var
        self.p_cov[3:6, 3:6] = np.eye(3) * vel_var
        self.p_cov[6:, 6:] = np.eye(3) * orien_var

        # Last updated timestamp (to compute the position
        # recovered by IMU velocity and acceleration, i.e.,
        # dead-reckoning)
        self.last_ts = 0

        # Sensor noise variances
        self.var_imu_acc = 0.01
        self.var_imu_gyro = 0.01
        # Motion model noise
        self.gnss_var = 100

        # Motion model noise Jacobian
        self.L_jacobian = np.zeros([9, 6])
        self.L_jacobian[3:, :] = np.eye(6)  # motion model noise jacobian

        # Measurement model Jacobian
        self.h_jac = np.zeros([3, 9])
        self.h_jac[:, :3] = np.eye(3)

        # Initialized
        self.n_gnss_taken = 0
        self.gnss_init_xyz = None
        self.initialized = False

        weak_self = weakref.ref(self)
        self.gnss_obj.gnss.listen(lambda data : ExtendedKalmanFilter.gnss_callback(weak_self, data))
        self.imu_obj.imu.listen(lambda data : ExtendedKalmanFilter.imu_callback(weak_self, data))

    @staticmethod
    def gnss_callback(weak_self, gnss_data):
        self = weak_self()
        if not self:
            return

        gnss_xyz = self.gnss_obj.get_xyz(gnss_data)
        if self.initialized:
            self.measurement_correction(gnss_xyz, self.gnss_var)
        else:
            self.initialize_pose(gnss_xyz)
    
    @staticmethod
    def imu_callback(weak_self, imu_data):
        self = weak_self()
        if not self:
            return

        if self.initialized:
            self.imu_prediction(imu_data)
        else:
            self.imu_obj.set_last_ts(imu_data)


    def initialize_pose(self, gnss_xyz, samples_to_use=10):
        x, y, z = gnss_xyz
        if self.gnss_init_xyz is None:
            self.gnss_init_xyz = np.array([x, y, z])
        else:
            self.gnss_init_xyz[0] += x
            self.gnss_init_xyz[1] += y
            self.gnss_init_xyz[2] += z
        self.n_gnss_taken += 1

        if self.n_gnss_taken == samples_to_use:
            self.gnss_init_xyz /= samples_to_use
            self.p[:, 0] = self.gnss_init_xyz
            self.q[:, 0] = Quaternion().to_numpy()
            
            self.initialized = True

    def measurement_correction(self, xyz, sensor_var):
        # Global position
        x, y, z = xyz

        R_cov = sensor_var * np.eye(3)

        # Kalman gain
        K = self.p_cov @ self.h_jac.T @ (np.linalg.inv(self.h_jac @ self.p_cov @ self.h_jac.T + R_cov))

        # Compute the error state
        delta_x = K @ (np.array([x, y, z])[:, None] - self.p)

        # Correction
        self.p = self.p + delta_x[:3]
        self.v = self.v + delta_x[3:6]
        delta_q = Quaternion(axis_angle=angle_normalize(delta_x[6:]))
        self.q = delta_q.quat_mult_left(self.q)

        # Corrected covariance
        self.p_cov = (np.identity(9) - K @ self.h_jac) @ self.p_cov

    def imu_prediction(self, imu_data):
        self.p, self.v, self.q, self.p_cov = self.imu_obj.imu_odometry(imu_data, self.p, self.v, self.q, self.p_cov, self.L_jacobian)
        
    
    def get_location(self):
        """Return the estimated vehicle location
        :return: x, y, z position
        :rtype: list
        """
        return self.p.reshape(-1).tolist()