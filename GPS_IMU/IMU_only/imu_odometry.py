from matplotlib.pyplot import axis
import numpy as np

from rotations import Quaternion, omega, skew_symmetric, angle_normalize
from scipy.spatial.transform import Rotation

class IMUOdometry:
    def __init__(self):
        # State (position, velocity and orientation)
        self.p = np.zeros([3, 1])
        self.v = np.zeros([3, 1])
        self.q = np.zeros([4, 1])  # quaternion

        # State covariance
        self.p_cov = np.zeros([9, 9])
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

        # Gravity
        self.g = np.array([0, 0, -9.81]).reshape(3, 1)

        # Sensor noise variances
        self.var_imu_acc = 0.01
        self.var_imu_gyro = 0.01

        # Motion model noise
        self.var_gnss = np.eye(3) * 100

        # Motion model noise Jacobian
        self.l_jac = np.zeros([9, 6])
        self.l_jac[3:, :] = np.eye(6)   # motion model noise jacobian

        # Measurement model Jacobian
        self.h_jac = np.zeros([3, 9])
        self.h_jac[:, :3] = np.eye(3)

        # Initialized
        self.n_gnss_taken = 0
        self.initialized = False


    def initialize_with_ground_truth(self, pose, vel, T_init, timestamp):
        self.p[:, 0] = np.array([pose.x, pose.y, pose.z])
        self.v[:, 0] = np.array([vel.x, vel.y, vel.z])
        #self.q[:, 0] = Quaternion().to_numpy()
        '''rot_matrix = T_init[:3, :3]
        rotation = Rotation.from_matrix(rot_matrix)
        self.q[:, 0] = rotation.as_quat()'''

        rotation = T_init.rotation
        roll = rotation.roll
        pitch = rotation.pitch
        yaw = rotation.yaw
        self.q[:, 0] = Quaternion(euler=[roll, pitch, yaw]).to_numpy()

        self.initialized = True

    def set_last_ts(self, imu):
        self.last_ts = imu.timestamp

    def get_location(self):
        """Return the estimated vehicle location

        :return: x, y, z position
        :rtype: list
        """
        return self.p.reshape(-1).tolist()

    def predict_state_with_imu(self, imu):
        # IMU acceleration and velocity
        imu_f = np.array([imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]).reshape(3, 1)
        imu_w = np.array([imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]).reshape(3, 1)

        # IMU sampling time
        delta_t = imu.timestamp - self.last_ts
        self.last_ts = imu.timestamp

        # Update state with imu
        R = Quaternion(*self.q).to_mat()
        #print(delta_t)
        self.p = self.p + delta_t * self.v + 0.5 * delta_t * delta_t * (R @ imu_f + self.g)
        self.v = self.v + delta_t * (R @ imu_f + self.g)
        self.q = omega(imu_w, delta_t) @ self.q

        # Update covariance
        F = self.calculate_motion_model_jacobian(R, imu_f, delta_t)
        Q = self.calculate_imu_noise(delta_t)
        self.p_cov = F @ self.p_cov @ F.T + self.l_jac @ Q @ self.l_jac.T

    def calculate_motion_model_jacobian(self, R, imu_f, delta_t):
        F = np.eye(9)
        F[:3, 3:6] = np.eye(3) * delta_t
        F[3:6, 6:] = -skew_symmetric(R @ imu_f) * delta_t

        return F

    def calculate_imu_noise(self, delta_t):
        Q = np.eye(6)
        Q[:3, :3] *= delta_t * delta_t * self.var_imu_acc
        Q[3:, 3:] *= delta_t * delta_t * self.var_imu_gyro

        return Q