from numpy import eye, array, transpose, linalg
from math import sqrt, pi, sin, tan, cos
from . import GeometryCalculator as GeCa
from scipy.spatial.transform import Rotation
from copy import deepcopy


class KalmanFilter:
    """Class containing Kalman filter operation for single IMU
       inside this class all angular variables are in radians or radians/s"""

    def __init__(self, g_in=9.8159, dt=0.001):
        # Initialise matrices and variables
        self.C = array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.P = eye(4) * 1e-8  # Estimate initial covariance (properly set in "set_state")
        self.Q = eye(4) * 6e-4  # How bad is model data
        self.R = eye(2) * 2e-7  # How bad is sensor data (changed in function of acceleration)

        self.state_estimate = array([[0], [0], [0], [0]])
        self.g = g_in
        self.phi_hat = 0
        self.theta_hat = 0
        self.psi_hat = 0.0
        self.dt = dt

    def set_state(self, init_cov=1e-8,
                  init_phi_hat=0.0, init_theta_hat=0.0, init_psi_hat=0.0):
        self.state_estimate[0] = init_phi_hat
        self.state_estimate[2] = init_theta_hat
        self.psi_hat = init_psi_hat
        # self.int_theta = init_theta
        # self.int_phi = init_phi
        # self.int_psi = init_psi
        if init_cov is not None:
            self.P = eye(4) * init_cov  # Estimate initial covariance
        pass

    def iteration(self, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z):
        phi_acc, theta_acc = GeCa.get_acc_angles(acc_x, acc_y, acc_z)

        # Get current total acceleration
        g_now = sqrt(acc_x ** 2.0 + acc_y ** 2.0 + acc_z ** 2.0)

        # Calculate and apply new measurement covariance
        # if abs(g_now - self.g) < 0.1:
        #     self.R = eye(2) * 2e-4
        # else:
        #     #    self.R = eye(2) * 10000000000
        self.R = eye(2) * (2e-4 * (1 + (g_now - self.g * 100) ** 2))
        # Get gyro measurements and calculate Euler angle derivatives
        [p, q, r] = [gyr_x * pi / 180,
                     gyr_y * pi / 180,
                     gyr_z * pi / 180]
        phi_dot = p + sin(self.phi_hat) * tan(self.theta_hat) * q + cos(self.phi_hat) * tan(
            self.theta_hat) * r
        theta_dot = cos(self.phi_hat) * q - sin(self.phi_hat) * r
        psi_dot = sin(self.phi_hat) / cos(self.theta_hat) * q + cos(self.phi_hat) / cos(
            self.theta_hat) * r
        # Kalman filter (model matrices)
        a = array([[1, -self.dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -self.dt], [0, 0, 0, 1]])  # Fk
        b = array([[self.dt, 0], [0, 0], [0, self.dt], [0, 0]])  # Bk

        gyro_input = array([[phi_dot], [theta_dot]])  # uk
        self.state_estimate = a.dot(self.state_estimate) + b.dot(gyro_input)
        self.P = a.dot(self.P.dot(transpose(a))) + self.Q

        measurement = array([[phi_acc], [theta_acc]])  # zk
        y_tilde = measurement - self.C.dot(self.state_estimate)  # yk
        s = self.R + self.C.dot(self.P.dot(transpose(self.C)))  # Sk
        k = self.P.dot(transpose(self.C).dot(linalg.inv(s)))  # Kk
        self.state_estimate = self.state_estimate + k.dot(y_tilde)  # xk
        self.P = (eye(4) - k.dot(self.C)).dot(self.P)  # Pk

        self.phi_hat = self.state_estimate[0][0]
        self.theta_hat = self.state_estimate[2][0]
        self.psi_hat = self.psi_hat + psi_dot * self.dt
        # self.int_theta = self.int_theta + theta_dot * self.dt
        # self.int_phi = self.int_phi + phi_dot * self.dt
        # self.int_psi = self.int_psi + psi_dot * self.dt

        return

    def apply_kalman(self, imu_in_list: list[dict]):
        """'imu_input': [{'mean_time_rel': 0.0,
                       'X_acc': 0.0, 'Y_acc': 0.0, 'Z_acc': 0.0,
                       'X_gyr': 0.0, 'Y_gyr': 0.0, 'Z_gyr': 0.0}]"""
        for row in imu_in_list:
            self.iteration(acc_x=row['X_acc'],
                           acc_y=row['Y_acc'],
                           acc_z=row['Z_acc'],
                           gyr_x=row['X_gyr'],
                           gyr_y=row['Y_gyr'],
                           gyr_z=row['Z_gyr'])

            row['theta'] = self.theta_hat * 180 / pi
            row['phi'] = self.phi_hat * 180 / pi
            row['psi'] = self.psi_hat * 180 / pi

        return imu_in_list


class LinearIntegrator:
    """Class for calculating position from acceleration data"""

    def __init__(self, initial_state=None, dt=0.001, g_in=9.8159):
        if initial_state is None:
            euler_temp = {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
            rot = Rotation.from_euler('ZYX', [euler_temp['psi'], euler_temp['theta'], euler_temp['phi']],
                                      degrees=True)
            rot_vect = rot.as_rotvec()
            initial_state = {'mean_time_rel': 0.0,
                             'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_speed': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_rot_vect': rot_vect,
                             'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
                             }
        self.initial_state = deepcopy(initial_state)
        self.dt = dt
        self.g = g_in
        self.end_state = None

    def set_state(self, state_input=None):
        if state_input is None:
            raise ValueError('No initial state given!')
        if type(state_input) == dict:
            if state_input['mean_time_rel'] is not None:
                self.initial_state['mean_time_rel'] = state_input['mean_time_rel']
            if state_input['dev_pos'] is not None:
                self.initial_state['dev_pos'] = deepcopy(state_input['dev_pos'])
            if state_input['dev_speed'] is not None:
                self.initial_state['dev_speed'] = deepcopy(state_input['dev_speed'])
            if state_input['dev_euler_ang'] is not None:
                self.initial_state['dev_euler_ang'] = deepcopy(state_input['dev_euler_ang'])
                # create rot_vect
                rot = Rotation.from_euler('ZYX',
                                          [self.initial_state['dev_euler_ang']['psi'],
                                           self.initial_state['dev_euler_ang']['theta'],
                                           self.initial_state['dev_euler_ang']['phi']],
                                          degrees=True)
                self.initial_state['dev_rot_vect'] = rot.as_rotvec()
            else:
                if state_input['dev_rot_vect'] is not None:
                    self.initial_state['dev_rot_vect'] = deepcopy(state_input['dev_rot_vect'])
                    # recreate euler angles
                    rot = Rotation.from_rotvec(self.initial_state['dev_rot_vect'])
                    self.initial_state['dev_euler_ang'] = rot.as_euler('ZYX', degrees=True)
        else:
            raise ValueError('Wrong initial state file format!')

    def integrate(self, input_list: list = None, initial_state: dict = None, degrees=True, auto_iterate=True):
        """

        @param degrees: True - degrees, False - radians
        @param input_list: [{'mean_time_rel': 0.0,
                             'X_acc': 0.0, 'Y_acc': 0.0, 'Z_acc': 0.0,
                             'X_gyr': 0.0, 'Y_gyr': 0.0, 'Z_gyr': 0.0,
                             'theta': 0.0, 'phi': 0.0,   'psi': 0.0}]
        @param initial_state: {'mean_time_rel': 0.0,
                             'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_speed': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_rot_vect': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
                             }
        @return: {{'mean_time_rel': 0.0,
                             'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_rot_vect': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                             'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
                             }
        """
        if initial_state is not None:
            self.set_state(deepcopy(initial_state))

        # integrate acceleration
        acc_e = GeCa.rotate_acc(input_list)
        temp_state = deepcopy(self.initial_state)
        for row in acc_e:
            temp_state['dev_speed']['X'] += row['X_acc_e'] * self.dt
            temp_state['dev_speed']['Y'] += row['Y_acc_e'] * self.dt
            temp_state['dev_speed']['Z'] += (row['Z_acc_e'] - self.g) * self.dt
            temp_state['dev_pos']['X'] += temp_state['dev_speed']['X'] * self.dt
            temp_state['dev_pos']['Y'] += temp_state['dev_speed']['Y'] * self.dt
            temp_state['dev_pos']['Z'] += temp_state['dev_speed']['Z'] * self.dt

        # update rot_vect
        rot = Rotation.from_euler('ZYX', [input_list[-1]['psi'], input_list[-1]['theta'], input_list[-1]['phi']],
                                  degrees=degrees)
        rot_vect = rot.as_rotvec()
        temp_state['dev_rot_vect'] = {'X': rot_vect[0], 'Y': rot_vect[1], 'Z': rot_vect[2]}
        temp_state['dev_euler_ang'] = {'psi': input_list[-1]['psi'],
                                       'theta': input_list[-1]['theta'],
                                       'phi': input_list[-1]['phi']}
        temp_state['mean_time_rel'] = input_list[-1]['mean_time_rel']

        self.end_state = temp_state
        if auto_iterate:
            self.initial_state = deepcopy(temp_state)
        return deepcopy(self.end_state)


class DynamicsModel:
    """Class containing Kalman filter along all other dynamics calculations"""

    def __init__(self, dt_in=0.001, g_in=9.8159):
        self.kalman = KalmanFilter(g_in=g_in, dt=dt_in)
        self.integrator = LinearIntegrator(dt=dt_in)

    def get_next_pos(self, imu_input, prev_state=None):
        if prev_state is not None:
            self.kalman.set_state(init_phi_hat=prev_state['dev_euler_ang']['phi'],
                                  init_theta_hat=prev_state['dev_euler_ang']['theta'],
                                  init_psi_hat=prev_state['dev_euler_ang']['psi'])
        imu_input = self.kalman.apply_kalman(imu_in_list=imu_input)
        return self.integrator.integrate(input_list=imu_input, initial_state=prev_state, degrees=True)

    def write_state(self, state_in=None, zero_point=False):
        """
        Set Kalman State with covariance reinitialisation
        @param zero_point: Determines initial covariance applied
        @param state_in: {'mean_time_rel': 0.0,
                        'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                        'dev_speed': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                        'dev_rot_vect': None,
                        'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
                        }"""
        if state_in is None:
            state_in = {'mean_time_rel': None,
                        'dev_pos': None,
                        'dev_speed': None,
                        'dev_rot_vect': None,
                        'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
                        }

        self.kalman.set_state(init_cov=0 if zero_point else None,
                              init_phi_hat=state_in['dev_euler_ang']['phi'],
                              init_theta_hat=state_in['dev_euler_ang']['theta'],
                              init_psi_hat=state_in['dev_euler_ang']['psi'])

        self.integrator.set_state(state_input=state_in)
