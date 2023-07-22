from numpy import eye, array, transpose, linalg, nan
from math import sqrt, pi, sin, tan, cos


class KalmanFilter:
    """Class containing Kalman filter operation for single IMU"""

    def __init__(self, g_in=9.8159):
        # Initialise matrices and variables
        self.C = array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.P = eye(4) * 1e-8  # Estimate initial covariance (properly set in "set_state")
        self.Q = eye(4) * 6e-4  # How bad is model data
        self.R = eye(2) * 2e-3  # How bad is sensor data (changed in function of acceleration)

        self.state_estimate = array([[0], [0], [0], [0]])
        self.g = g_in
        self.phi_hat = 0
        self.theta_hat = 0
        self.psi_hat = 0.0
        self.int_theta = 0.0
        self.int_phi = 0.0
        # Calculate accelerometer offsets
        self.phi_offset = 0.0
        self.theta_offset = 0.0
        self.dt = 0.001

    def iteration(self, row, extra=False, acc_names=None):
        if acc_names is None:
            acc_names = ['acc_X_t', 'acc_Y_t', 'acc_Z_t']
        [phi_acc, theta_acc] = [row['acc_phi'], row['acc_theta']]
        phi_acc -= self.phi_offset
        theta_acc -= self.theta_offset
        # Get current total acceleration
        g_now = sqrt(row[acc_names[0]] ** 2.0 + row[acc_names[1]] ** 2.0 + row[acc_names[2]] ** 2.0)

        # Calculate and apply new measurement covariance
        if abs(g_now - self.g) < 0.001:
            self.R = eye(2) * 4e-3
        else:
            #    self.R = eye(2) * 10000000000
            self.R = eye(2) * (4e-3 * (1 + (g_now - self.g) ** 2 * 100))
        # Get gyro measurements and calculate Euler angle derivatives
        [p, q, r] = [row['gyro_X_b'] * pi / 180,
                     row['gyro_Y_b'] * pi / 180,
                     row['gyro_Z_b'] * pi / 180]
        phi_dot = p + sin(self.phi_hat) * tan(self.theta_hat) * q + cos(self.phi_hat) * tan(
            self.theta_hat) * r
        theta_dot = cos(self.phi_hat) * q - sin(self.phi_hat) * r
        psi_dot = sin(self.phi_hat) / cos(self.theta_hat) * q + cos(self.phi_hat) / cos(
            self.theta_hat) * r
        # Kalman filter (model matrices)
        A = array([[1, -self.dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -self.dt], [0, 0, 0, 1]])  # Fk
        B = array([[self.dt, 0], [0, 0], [0, self.dt], [0, 0]])  # Bk

        gyro_input = array([[phi_dot], [theta_dot]])  # uk
        self.state_estimate = A.dot(self.state_estimate) + B.dot(gyro_input)
        self.P = A.dot(self.P.dot(transpose(A))) + self.Q

        measurement = array([[phi_acc], [theta_acc]])  # zk
        y_tilde = measurement - self.C.dot(self.state_estimate)  # yk
        S = self.R + self.C.dot(self.P.dot(transpose(self.C)))  # Sk
        K = self.P.dot(transpose(self.C).dot(linalg.inv(S)))  # Kk
        self.state_estimate = self.state_estimate + K.dot(y_tilde)  # xk
        self.P = (eye(4) - K.dot(self.C)).dot(self.P)  # Pk

        self.phi_hat = self.state_estimate[0]
        self.theta_hat = self.state_estimate[2]
        self.psi_hat = self.psi_hat + psi_dot * self.dt
        self.int_theta = self.int_theta + theta_dot * self.dt
        self.int_phi = self.int_phi + phi_dot * self.dt

        row['phi_hat'] = self.state_estimate[0]
        row['theta_hat'] = self.state_estimate[2]
        row['psi_hat'] = self.psi_hat
        row['g_sensor'] = g_now
        if extra:
            row['P_0_0'] = self.P[0, 0]
            row['P_1_1'] = self.P[1, 1]
            row['P_2_2'] = self.P[2, 2]
            row['P_3_3'] = self.P[3, 3]
            row['R'] = self.R[0, 0]
            row['int_theta'] = self.int_theta
            row['int_phi'] = self.int_phi

        return row

    def apply_kalman(self, df_in, extra=False, acc_names=None):
        if acc_names is None:
            acc_names = ['acc_X_t', 'acc_Y_t', 'acc_Z_t']
        df_in['phi_hat'] = nan
        df_in['psi_hat'] = nan
        df_in['theta_hat'] = nan
        df_in['g_sensor'] = nan
        if extra:
            df_in['P_0_0'] = nan
            df_in['P_1_1'] = nan
            df_in['P_2_2'] = nan
            df_in['P_3_3'] = nan
            df_in['R'] = nan
            df_in['int_theta'] = nan
            df_in['int_phi'] = nan

        self.C = array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.P = eye(4) * 1e-8  # Estimate initial covariance (properly set in "set_state")
        self.Q = eye(4) * 6e-4  # How bad is model data
        self.R = eye(2) * 2e-3  # How bad is sensor data (changed in function of acceleration)
        # Initial estimates
        # self.state_estimate = array([[20 * 3.14 / 180], [0], [20 * 3.14 / 180], [0]])
        self.state_estimate = array([[0], [0], [0], [0]])
        # self.phi_hat = 0.0
        # self.theta_hat = 0.0
        # self.psi_hat = 0.0
        # Calculate accelerometer offsets
        self.phi_offset = 0.0
        self.theta_offset = 0.0
        self.dt = 0.001

        return df_in.apply(lambda row: self.iteration(row, extra=extra, acc_names=acc_names), axis=1)



