# Class coping with all data analysis functionalities
# 1. Sensor data initial bias compensation
# 2. Sensor data coordinates translation

from math import pi
from numpy import array
from DataAnalysisClasses.SensorCurve3D import SensorCurve3D
from DataAnalysisClasses.AngleCalculator import AngleTranslator
from DataAnalysisClasses.KalmanFilter import KalmanFilter
from DataAnalysisClasses.BiasCompensator import BiasCompensator
from ComunicationClasses.Database_collector import Database_collector
import matplotlib.pyplot as plt
import pandas as pd

if __name__ == '__main__':

    print('Testing connection')
    db_host_address = 'localhost'
    print('Server IP: ' + db_host_address)
    db_client = Database_collector(host=db_host_address)
    if not db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')

    print('List of databases:')
    print(db_client.get_db_list())

    print('List of measurements:')
    print(db_client.get_measurements_list('scan_sensor_test'))

    bias = BiasCompensator('ISM_330', 'linear')
    translator = AngleTranslator()
    kalman = KalmanFilter()
    data_temp = db_client.get_measurement_data('scan_sensor_test', 'ISM_330_raw', 1644518082462, 1644518106858).dropna()
    data = data_temp.copy()
    data['mean_acc_X'] = data_temp['mean_acc_Z']
    data['mean_acc_Z'] = data_temp['mean_acc_X'] * (-1.0)
    data['mean_gyro_X'] = data_temp['mean_gyro_Z']
    data['mean_gyro_Z'] = data_temp['mean_gyro_X'] * (-1.0)

    # apply bias compensation to gyroscopes
    print(bias.bias_const)
    bias.set_gyro_const_bias(8000, data)
    # bias.set_acc_const_bias(8000, data)
    print(bias.bias_const)

    data = data.drop(data.index[16000:])
    data = data.drop(data.index[:8000])

    with pd.option_context('display.max_rows', None, 'display.max_columns', None,
                           'display.precision', 3):
        print(data.describe())
    data = bias.apply_bias(data)
    # calculate phi and theta from accelerometers data
    data = translator.calculate_angles_1(data)
    # apply "Kalman filter" to phi and theta calculations (also get psi as an integral)
    data = kalman.apply_kalman(data)

    # rotate sensors accelerations from sensor frame to earths frame using quaternions
    data = translator.rotate_acc(data)

    # get points 3D
    curve = SensorCurve3D()

    fun_iter = 0

    curve.fill_curve(data, 0.001, 9.8159, None)

    speed_corrections = {'x': curve.x_speed_last / 8000, 'y': curve.y_speed_last / 8000, 'z': curve.z_speed_last / 8000}
    curve.fill_curve(data, 0.001, 9.8159, speed_corrections)


    def opt_iteration(bias_tab):
        global fun_iter
        global data
        global curve
        global kalman
        fun_iter += 1

        print('Iteration: ', fun_iter)
        print('Input: ', bias_tab)
        # apply new bias to input data
        bias.change_bias_acc_linear(bias_tab[0], bias_tab[1], bias_tab[2], bias_tab[3], bias_tab[4], bias_tab[5])
        # bias.change_bias_gyro_const(bias_tab[3], bias_tab[4], bias_tab[5])
        # g_temp= bias_tab[6]
        g_temp = 9.8159
        bias.apply_bias(data)
        # recalculate Kalman
        data = translator.calculate_angles_1(data)
        data = kalman.apply_kalman(data)
        data = translator.rotate_acc(data)
        curve.fill_curve(data, 0.001, g_temp)
        speed_arg = curve.x_speed_last ** 2 + curve.y_speed_last ** 2 + curve.z_speed_last ** 2
        # pos_arg = curve.x_point_last ** 2 + curve.y_point_last ** 2 + curve.z_point_last ** 2
        # deg_arg = kalman.psi_hat ** 2
        acc_arg = (curve.acc_last - 9.8159) ** 2
        out = speed_arg + acc_arg * 10000  # + pos_arg# + deg_arg
        print('Output: ', out, ' G_fin: ', curve.acc_last)
        return out


    # run single opt iteration
    x_0 = array([bias.bias_linear['a_x_acc'], bias.bias_linear['a_y_acc'], bias.bias_linear['a_z_acc'],
                 bias.bias_linear['b_x_acc'], bias.bias_linear['b_y_acc'], bias.bias_linear['b_z_acc']])
    # x_0 = np.array([bias.bias_const['x_acc'], bias.bias_const['y_acc'], bias.bias_const['z_acc']])
    #                bias.bias_const['x_gyro'], bias.bias_const['y_gyro'], bias.bias_const['z_gyro']])
    # x_0 = np.array([-0.6,-0.6,-0.6,-0.4,-0.4,-0.4])
    # func = opt_iteration(x_0)
    # print(func)
    # minimize(opt_iteration, x_0, method='L-BFGS-B', bounds=[(-0.6, 0.6), (-0.6, 0.6), (-0.6, 0.6) ,(-0.4, 0.4), (-0.4, 0.4), (-0.4, 0.4)],
    #         options={'maxfun': 100, 'eps': 0.0001, 'ftol': 0.001, 'iprint': 0})

    # minimize(opt_iteration, x_0, method='L-BFGS-B',  # method='TNC',
    #         bounds=[(0.98, 1.02), (0.98, 1.02), (0.98, 1.02), (-0.15, 0.15), (-0.15, 0.15), (-0.15, 0.15)],
    #         options={'maxiter': 100, 'eps': 0.0001, 'ftol': 0.001})
    data.set_index('mean_time_rel', inplace=True)
    with pd.option_context('display.max_rows', None, 'display.max_columns', None,
                           'display.precision', 3,
                           ):
        print(data.describe())

    # data.drop('')
    plt.style.use('dark_background')
    figure, axis = plt.subplots(3, 2)
    axis[0, 0].plot(data.index, data['mean_acc_X_b'], label='sensor X')
    axis[0, 0].plot(data.index, data['mean_acc_Y_b'], label='sensor Y')
    axis[0, 0].plot(data.index, data['mean_acc_Z_b'], label='sensor Z')
    axis[0, 0].plot(data.index, data['mean_acc_X'], label='sensor X raw')
    axis[0, 0].plot(data.index, data['mean_acc_Y'], label='sensor Y raw')
    axis[0, 0].plot(data.index, data['mean_acc_Z'], label='sensor Z raw')
    axis[0, 0].legend(loc='upper left')
    axis[0, 0].set_xlabel('time [ms]')
    axis[0, 0].set_ylabel('acceleration [m/s^2]')
    axis[0, 1].plot(data.index, data['mean_gyro_X_b'], label='sensor X')
    axis[0, 1].plot(data.index, data['mean_gyro_Y_b'], label='sensor Y')
    axis[0, 1].plot(data.index, data['mean_gyro_Z_b'], label='sensor Z')
    axis[0, 1].legend(loc='upper left')
    axis[0, 1].set_xlabel('time [ms]')
    axis[0, 1].set_ylabel('rotation [deg/s]')

    axis[1, 0].plot(data.index, data['g_sensor'], label='sensor acceleration')
    axis[1, 0].plot(data.index, data['g_ground'], label='ground acceleration')
    axis[1, 0].set_xlabel('time [ms]')
    axis[1, 0].set_ylabel('acceleration [m/s^2]')
    axis[1, 0].legend(loc='upper left')

    axis[1, 1].plot(data.index, data['acc_theta'] * 180 / pi, label='theta (acceleration)')
    axis[1, 1].plot(data.index, data['theta_hat'] * 180 / pi, label='theta (Kalman)')
    axis[1, 1].plot(data.index, data['acc_phi'] * 180 / pi, label='phi (acceleration)')
    axis[1, 1].plot(data.index, data['phi_hat'] * 180 / pi, label='phi (Kalman)')
    axis[1, 1].set_xlabel('time [ms]')
    axis[1, 1].set_ylabel('angle [deg]')
    axis[1, 1].legend(loc='upper left')
    axis[2, 0].plot(data.index, data['psi_hat'] * 180 / pi, label='psi (integration)')
    axis[2, 0].set_xlabel('time [ms]')
    axis[2, 0].set_ylabel('angle [deg]')
    axis[2, 0].legend(loc='upper left')
    axis[2, 1].plot(data.index, data['acc_X_e'], label='ground X')
    axis[2, 1].plot(data.index, data['acc_Y_e'], label='ground Y')
    # axis[2, 1].plot(data.index, data['acc_Z_e'] - 9.8159, label='ground Z')
    axis[2, 1].plot(data.index, data['acc_Z_e'], label='ground Z')
    axis[2, 1].set_xlabel('time [ms]')
    axis[2, 1].set_ylabel('acceleration [m/s^2]')
    axis[2, 1].legend(loc='upper left')
    ax1 = plt.figure().add_subplot(projection='3d')
    ax1.plot(curve.x_point_list, curve.y_point_list, curve.z_point_list, label='position curve')
    ax1.set_xlabel('position X [m]')
    ax1.set_ylabel('position Y [m]')
    ax1.set_zlabel('position Z [m]')

    ax2 = plt.figure().add_subplot(projection='3d')
    ax2.plot(curve.x_speed_list, curve.y_speed_list, curve.z_speed_list, label='speed curve')
    ax2.set_xlabel('speed X [m/s]')
    ax2.set_ylabel('speed Y [m/s]')
    ax2.set_zlabel('speed Z [m/s]')
    # data.to_csv('output.csv')
    plt.show()

    # print('Lack of testing code')
