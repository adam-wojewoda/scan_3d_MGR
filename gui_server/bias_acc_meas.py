import ComunicationClasses.Database_collector as DaCo
from tqdm import tqdm
import matplotlib.pyplot as plt
from math import sqrt
import numpy as np
from scipy.optimize import minimize

if __name__ == '__main__':
    print('Testing connection')
    db_host_address = 'localhost'
    print('Server IP: ' + db_host_address)
    db_client = DaCo.Database_collector(host=db_host_address)
    if not db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')

    print('List of databases:')
    print(db_client.get_db_list())

    print('List of measurements:')
    print(db_client.get_measurements_list('scan_sensor_test'))

    # Using readlines()
    file1 = open('Data_doc/cal_times.txt', 'r')
    Lines = file1.readlines()

    ISM_sens = 'ISM_330'
    MPU_sens = 'MPU_9255'
    sensor_name = MPU_sens
    sensor_name_raw = sensor_name + '_raw'
    # Get sensor times
    sensor_time_list = []
    for line in Lines:
        l_tmp = (line.strip()).split()
        if ISM_sens == sensor_name:
            sensor_time_list.append({'start': float(l_tmp[1]), 'stop': float(l_tmp[3])})
        else:
            sensor_time_list.append({'start': float(l_tmp[5]), 'stop': float(l_tmp[7])})

    print(sensor_name)
    # print(sensor_time_list)

    # meas_list = []
    mean_list = {'mean_acc_X': [],
                 'mean_acc_Y': [],
                 'mean_acc_Z': [],
                 'mean_gyro_X': [],
                 'mean_gyro_Y': [],
                 'mean_gyro_Z': []}
    g_tot = []

    for meas in tqdm(sensor_time_list):
        # print(int((meas['start']+3600)*1000),int((meas['stop']+3600)*1000))
        new_meas = (
            db_client.get_measurement_data('scan_sensor_test', sensor_name_raw, (int(meas['start'] + 3600) * 1000),
                                           (int(meas['stop'] + 3600) * 1000)).dropna()).mean()
        # meas_list.append(new_meas)
        g_tot.append(sqrt(new_meas['mean_acc_X'] ** 2 + new_meas['mean_acc_Y'] ** 2 + new_meas['mean_acc_Z'] ** 2))
        for key in mean_list:
            mean_list[key].append(new_meas[key])
        # print(new_meas.describe())

    print('g_tot list:')
    print(g_tot)
    print('Mean acc val: ', np.mean(g_tot))

    if False:
        plt.style.use('dark_background')

        figure, axis = plt.subplots(2, 3)
        axis[0, 0].scatter(mean_list['mean_acc_X'], mean_list['mean_gyro_X'])
        axis[0, 0].scatter(mean_list['mean_acc_X'], mean_list['mean_gyro_Y'])
        axis[0, 0].scatter(mean_list['mean_acc_X'], mean_list['mean_gyro_Z'])

        axis[0, 1].scatter(mean_list['mean_acc_Y'], mean_list['mean_gyro_X'])
        axis[0, 1].scatter(mean_list['mean_acc_Y'], mean_list['mean_gyro_Y'])
        axis[0, 1].scatter(mean_list['mean_acc_Y'], mean_list['mean_gyro_Z'])

        axis[0, 2].scatter(mean_list['mean_acc_Z'], mean_list['mean_gyro_X'])
        axis[0, 2].scatter(mean_list['mean_acc_Z'], mean_list['mean_gyro_Y'])
        axis[0, 2].scatter(mean_list['mean_acc_Z'], mean_list['mean_gyro_Z'])

        axis[1, 0].scatter(mean_list['mean_gyro_X'], mean_list['mean_acc_X'])
        axis[1, 0].scatter(mean_list['mean_gyro_X'], mean_list['mean_acc_Y'])
        axis[1, 0].scatter(mean_list['mean_gyro_X'], mean_list['mean_acc_Z'])

        axis[1, 1].scatter(mean_list['mean_gyro_Y'], mean_list['mean_acc_X'])
        axis[1, 1].scatter(mean_list['mean_gyro_Y'], mean_list['mean_acc_Y'])
        axis[1, 1].scatter(mean_list['mean_gyro_Y'], mean_list['mean_acc_Z'])

        axis[1, 2].scatter(mean_list['mean_gyro_Z'], mean_list['mean_acc_X'])
        axis[1, 2].scatter(mean_list['mean_gyro_Z'], mean_list['mean_acc_Y'])
        axis[1, 2].scatter(mean_list['mean_gyro_Z'], mean_list['mean_acc_Z'])

        ax1 = plt.figure().add_subplot(projection='3d')
        ax1.scatter(mean_list['mean_acc_X'], mean_list['mean_acc_Y'], mean_list['mean_acc_Z'], label='sensor_acc')

        ax2 = plt.figure().add_subplot(projection='3d')
        ax2.scatter(mean_list['mean_gyro_X'], mean_list['mean_gyro_Y'], mean_list['mean_gyro_Z'], label='sensor_gyro')
        plt.show()

    # x = [1, 1, 1, 0, 0, 0]
    x = [0, 0, 0]


    def target_func(bias_tab):  # bias_tab ACC ax,ay,az,bx,by,bz
        global mean_list
        global x
        # ax = bias_tab[0]
        # ay = bias_tab[1]
        # az = bias_tab[2]
        # bx = bias_tab[3]
        # by = bias_tab[4]
        # bz = bias_tab[5]
        ax = 1.0
        ay = 1.0
        az = 1.0
        bx = bias_tab[0]
        by = bias_tab[1]
        bz = bias_tab[2]
        x = bias_tab
        acc_x_in = mean_list['mean_acc_X'][:]
        acc_y_in = mean_list['mean_acc_Y'][:]
        acc_z_in = mean_list['mean_acc_Z'][:]

        acc_x = [x * ax + bx for x in acc_x_in]
        acc_y = [y * ay + by for y in acc_y_in]
        acc_z = [z * az + bz for z in acc_z_in]

        g_tab = [np.sqrt(acc_x[i] ** 2 + acc_y[i] ** 2 + acc_z[i] ** 2) for i in range(
            min([len(acc_x), len(acc_y), len(acc_z)]))]

        std_out = np.std(g_tab)
        mean_out = np.mean(g_tab)

        print('Std: ', std_out)
        print('Mean: ', mean_out)
        return std_out / mean_out + ((9.8159 - mean_out) ** 2 * 50)


    # 9.8159
    # std = target_func([1, 1, 1, 0, 0, 0])
    # x_0 = [1, 1, 1, 0, 0, 0]
    x_0 = [0, 0, 0]
    minimize(target_func, np.array(x_0), method='L-BFGS-B',
             # bounds=[(0.9, 1.1), (0.9, 1.1), (0.9, 1.1), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)],
             bounds=[(-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)],
             options={'maxfun': 100000, 'eps': 0.0000001, 'ftol': 0.000001, 'iprint': 0})
    print('final compensation: ', x)
