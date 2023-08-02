import numpy as np
import math

from rotations import Quaternion, omega, skew_symmetric, angle_normalize

class IMU:
    def __init__(self, world, vehicle, blueprint_library, imu_transform):
        self.world = world
        # IMU parameters
        self.NOISE_STDDEV = 5e-5
        self.NOISE_BIAS = 1e-5
        self.IMU_FREQ = 200

        # Sensor noise variances
        self.var_imu_acc = 0.01
        self.var_imu_gyro = 0.01

        # imu timestamp
        self.last_ts = 0
        
        self.g = np.array([0, 0, -9.81]).reshape(3, 1)

        #gnss blueprint
        imu_bp = self.generate_IMU_bp(blueprint_library)

        # Attach IMU to vehicle
        self.imu = self.world.spawn_actor(
            blueprint=imu_bp,
            transform=imu_transform,
            attach_to=self.vehicle
        )

    def generate_IMU_bp(self, blueprint_library):

        """Generates a CARLA blueprint based on the script parameters"""
        # Sensor noise profile 
        NOISE_IMU_ACC_X_STDDEV = self.NOISE_STDDEV
        NOISE_IMU_ACC_Y_STDDEV = self.NOISE_STDDEV
        NOISE_IMU_ACC_Z_STDDEV = self.NOISE_STDDEV
        NOISE_IMU_GYRO_X_BIAS = self.NOISE_BIAS
        NOISE_IMU_GYRO_X_STDDEV = self.NOISE_STDDEV
        NOISE_IMU_GYRO_Y_BIAS = self.NOISE_BIAS
        NOISE_IMU_GYRO_Y_STDDEV = self.NOISE_STDDEV
        NOISE_IMU_GYRO_Z_BIAS = self.NOISE_BIAS
        NOISE_IMU_GYRO_Z_STDDEV = self.NOISE_STDDEV

        # initilize IMU sensor
        imu_bp = blueprint_library.filter("sensor.other.imu")[0]

        # Set sensors' noise
        imu_bp.set_attribute('noise_accel_stddev_x', str(NOISE_IMU_ACC_X_STDDEV))
        imu_bp.set_attribute('noise_accel_stddev_y', str(NOISE_IMU_ACC_Y_STDDEV))
        imu_bp.set_attribute('noise_accel_stddev_z', str(NOISE_IMU_ACC_Z_STDDEV))
        imu_bp.set_attribute('noise_gyro_stddev_x', str(NOISE_IMU_GYRO_X_STDDEV))
        imu_bp.set_attribute('noise_gyro_stddev_y', str(NOISE_IMU_GYRO_Y_STDDEV))
        imu_bp.set_attribute('noise_gyro_stddev_z', str(NOISE_IMU_GYRO_Z_STDDEV))
        imu_bp.set_attribute('noise_gyro_bias_x', str(NOISE_IMU_GYRO_X_BIAS))
        imu_bp.set_attribute('noise_gyro_bias_y', str(NOISE_IMU_GYRO_Y_BIAS))
        imu_bp.set_attribute('noise_gyro_bias_z', str(NOISE_IMU_GYRO_Z_BIAS))

        # Sensor sampling frequency
        imu_bp.set_attribute('sensor_tick', str(1.0 / self.IMU_FREQ))

        return imu_bp

    def set_last_ts(self, imu_data):
        self.last_ts = imu_data.timestamp

    def imu_odometry(self, imu_data, p, v, q, p_conv, L_jacobian):
        # IMU acceleration and velocity
        imu_f = np.array([imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z]).reshape(3, 1)
        imu_w = np.array([imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z]).reshape(3, 1)

        # IMU sampling time
        delta_t = imu_data.timestamp - self.last_ts
        self.last_ts = imu_data.timestamp

        # Update state with imu
        R = Quaternion(*q).to_mat()
        #print(delta_t)
        p = p + delta_t * v + 0.5 * delta_t * delta_t * (R @ imu_f + self.g)
        v = v + delta_t * (R @ imu_f + self.g)
        q = omega(imu_w, delta_t) @ q

        # Update covariance
        F = self.calculate_motion_model_jacobian(R, imu_f, delta_t)
        Q = self.calculate_imu_noise(delta_t)
        p_conv = F @ p_conv @ F.T + L_jacobian @ Q @ L_jacobian.T

        return p, v, q, p_conv

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