import ComunicationClasses.Database_collector as DaCo
from tqdm import tqdm
import matplotlib.pyplot as plt
from math import sqrt
import numpy as np
from scipy.optimize import minimize
from time import sleep
from scipy.spatial.transform import Rotation

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
    sensor_name1 = ISM_sens
    sensor_name1_raw = sensor_name1 + '_raw'
    sensor_name2 = MPU_sens
    sensor_name2_raw = sensor_name2 + '_raw'
    # Get sensor times
    sensor_time_list1 = []
    for line in Lines:
        l_tmp = (line.strip()).split()
        if ISM_sens == sensor_name1:
            sensor_time_list1.append({'start': float(l_tmp[1]), 'stop': float(l_tmp[3])})
        else:
            sensor_time_list1.append({'start': float(l_tmp[5]), 'stop': float(l_tmp[7])})

    sensor_time_list2 = []
    for line in Lines:
        l_tmp = (line.strip()).split()
        if ISM_sens == sensor_name2:
            sensor_time_list2.append({'start': float(l_tmp[1]), 'stop': float(l_tmp[3])})
        else:
            sensor_time_list2.append({'start': float(l_tmp[5]), 'stop': float(l_tmp[7])})

    # print(sensor_name)
    # print(sensor_time_list)

    # meas_list = []
    mean_list_ISM_raw = {'mean_acc_X': [],
                         'mean_acc_Y': [],
                         'mean_acc_Z': [],
                         'mean_gyro_X': [],
                         'mean_gyro_Y': [],
                         'mean_gyro_Z': []}
    mean_list_MPU_raw = {'mean_acc_X': [],
                         'mean_acc_Y': [],
                         'mean_acc_Z': [],
                         'mean_gyro_X': [],
                         'mean_gyro_Y': [],
                         'mean_gyro_Z': []}
    mean_list_ISM_target = {'mean_acc_X': [],
                            'mean_acc_Y': [],
                            'mean_acc_Z': [],
                            'mean_gyro_X': [],
                            'mean_gyro_Y': [],
                            'mean_gyro_Z': []}
    mean_list_ISM_rot = {'mean_acc_X': [],
                         'mean_acc_Y': [],
                         'mean_acc_Z': [],
                         'mean_gyro_X': [],
                         'mean_gyro_Y': [],
                         'mean_gyro_Z': []}
    mean_list_MPU_rot = {'mean_acc_X': [],
                         'mean_acc_Y': [],
                         'mean_acc_Z': [],
                         'mean_gyro_X': [],
                         'mean_gyro_Y': [],
                         'mean_gyro_Z': []}

    print('\nGetting ISM measurements mean values \n')
    for meas in tqdm(sensor_time_list1):
        # print(int((meas['start']+3600)*1000),int((meas['stop']+3600)*1000))
        new_meas = (
            db_client.get_measurement_data('scan_sensor_test', sensor_name1_raw, (int(meas['start'] + 3600) * 1000),
                                           (int(meas['stop'] + 3600) * 1000)).dropna()).mean()
        # meas_list.append(new_meas)
        for key in mean_list_ISM_raw:
            mean_list_ISM_raw[key].append(new_meas[key])

    sleep(0.01)
    print('\nGetting MPU measurements mean values \n')
    for meas in tqdm(sensor_time_list2):
        # print(int((meas['start']+3600)*1000),int((meas['stop']+3600)*1000))
        new_meas = (
            db_client.get_measurement_data('scan_sensor_test', sensor_name2_raw, (int(meas['start'] + 3600) * 1000),
                                           (int(meas['stop'] + 3600) * 1000)).dropna()).mean()
        # meas_list.append(new_meas)
        for key in mean_list_MPU_raw:
            mean_list_MPU_raw[key].append(new_meas[key])

    sleep(0.01)
    print('Calculating proper ISM values algebraically')
    mean_list_ISM_target['mean_acc_X'] = mean_list_ISM_raw['mean_acc_Z'][:]
    mean_list_ISM_target['mean_gyro_X'] = mean_list_ISM_raw['mean_gyro_Z'][:]
    mean_list_ISM_target['mean_acc_Y'] = mean_list_ISM_raw['mean_acc_Y'][:]
    mean_list_ISM_target['mean_gyro_Y'] = mean_list_ISM_raw['mean_gyro_Y'][:]
    mean_list_ISM_target['mean_acc_Z'] = [x * (-1.0) for x in mean_list_ISM_raw['mean_acc_X']]
    mean_list_ISM_target['mean_gyro_Z'] = [x * (-1.0) for x in mean_list_ISM_raw['mean_gyro_X']]

    print('Finding ratation by optimization')

    print('Searching ISM rotation')
    ISM_fin_rot = [0, 0, 0]
    mean_list_ISM_rot['mean_acc_X'] = mean_list_ISM_raw['mean_acc_X'][:]
    mean_list_ISM_rot['mean_acc_Y'] = mean_list_ISM_raw['mean_acc_Y'][:]
    mean_list_ISM_rot['mean_acc_Z'] = mean_list_ISM_raw['mean_acc_Z'][:]
    mean_list_ISM_rot['mean_gyro_X'] = mean_list_ISM_raw['mean_gyro_X'][:]
    mean_list_ISM_rot['mean_gyro_Y'] = mean_list_ISM_raw['mean_gyro_Y'][:]
    mean_list_ISM_rot['mean_gyro_Z'] = mean_list_ISM_raw['mean_gyro_Z'][:]


    def ism_find_rotation(rot_vect_in):
        global mean_list_ISM_target
        global mean_list_ISM_raw
        global mean_list_ISM_rot
        global ISM_fin_rot
        ISM_fin_rot = rot_vect_in
        rot = Rotation.from_euler('ZYX', [rot_vect_in[0], rot_vect_in[1], rot_vect_in[2]], degrees=True)
        error_sum = 0.0
        meas_len = len(mean_list_ISM_rot['mean_acc_X'])
        for i in range(meas_len):
            [ax_e, ay_e, az_e] = rot.apply([mean_list_ISM_raw['mean_acc_X'][i],
                                            mean_list_ISM_raw['mean_acc_Y'][i],
                                            mean_list_ISM_raw['mean_acc_Z'][i]], inverse=False)
            mean_list_ISM_rot['mean_acc_X'][i] = ax_e
            mean_list_ISM_rot['mean_acc_Y'][i] = ay_e
            mean_list_ISM_rot['mean_acc_Z'][i] = az_e

            error_sum = error_sum + ((ax_e - mean_list_ISM_target['mean_acc_X'][i]) ** 2 +
                                     (ay_e - mean_list_ISM_target['mean_acc_Y'][i]) ** 2 +
                                     (az_e - mean_list_ISM_target['mean_acc_Z'][i])) ** 2

            [ax_e, ay_e, az_e] = rot.apply([mean_list_ISM_raw['mean_gyro_X'][i],
                                            mean_list_ISM_raw['mean_gyro_Y'][i],
                                            mean_list_ISM_raw['mean_gyro_Z'][i]], inverse=False)
            mean_list_ISM_rot['mean_gyro_X'][i] = ax_e
            mean_list_ISM_rot['mean_gyro_Y'][i] = ay_e
            mean_list_ISM_rot['mean_gyro_Z'][i] = az_e

        return error_sum / meas_len


    x_0 = [0, 0, 0]
    minimize(ism_find_rotation, np.array(x_0), method='L-BFGS-B',
             # bounds=[(0.9, 1.1), (0.9, 1.1), (0.9, 1.1), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)],
             bounds=[(-180, 180), (-180, 180), (-180, 180)],
             options={'maxfun': 100000, 'eps': 0.0000001, 'ftol': 0.000001, 'iprint': -1})
    print('Rotation angles for ISM sensor: ', ISM_fin_rot)

    rotations_list = [
        'xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx',
        'XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'
    ]

    print('Searching MPU rotation')
    MPU_fin_rot = [0, 0, 0]
    mean_list_MPU_rot['mean_acc_X'] = mean_list_MPU_raw['mean_acc_X'][:]
    mean_list_MPU_rot['mean_acc_Y'] = mean_list_MPU_raw['mean_acc_Y'][:]
    mean_list_MPU_rot['mean_acc_Z'] = mean_list_MPU_raw['mean_acc_Z'][:]
    mean_list_MPU_rot['mean_gyro_X'] = mean_list_MPU_raw['mean_gyro_X'][:]
    mean_list_MPU_rot['mean_gyro_Y'] = mean_list_MPU_raw['mean_gyro_Y'][:]
    mean_list_MPU_rot['mean_gyro_Z'] = mean_list_MPU_raw['mean_gyro_Z'][:]


    def mpu_find_rotation(rot_vect_in):
        global mean_list_ISM_target
        global mean_list_MPU_raw
        global mean_list_MPU_rot
        global MPU_fin_rot
        MPU_fin_rot = rot_vect_in
        rot = Rotation.from_euler('XYZ', [rot_vect_in[0], rot_vect_in[1], rot_vect_in[2]], degrees=True)
        error_sum = 0.0
        meas_len = len(mean_list_MPU_rot['mean_acc_X'])
        for i in range(meas_len):
            [ax_e, ay_e, az_e] = rot.apply([mean_list_MPU_raw['mean_acc_X'][i],
                                            mean_list_MPU_raw['mean_acc_Y'][i],
                                            mean_list_MPU_raw['mean_acc_Z'][i]], inverse=False)
            mean_list_MPU_rot['mean_acc_X'][i] = ax_e
            mean_list_MPU_rot['mean_acc_Y'][i] = ay_e
            mean_list_MPU_rot['mean_acc_Z'][i] = az_e

            error_sum = error_sum + ((ax_e - mean_list_ISM_target['mean_acc_X'][i]) ** 2 +
                                     (ay_e - mean_list_ISM_target['mean_acc_Y'][i]) ** 2 +
                                     (az_e - mean_list_ISM_target['mean_acc_Z'][i])) ** 2

            [ax_e, ay_e, az_e] = rot.apply([mean_list_MPU_raw['mean_gyro_X'][i],
                                            mean_list_MPU_raw['mean_gyro_Y'][i],
                                            mean_list_MPU_raw['mean_gyro_Z'][i]], inverse=False)
            mean_list_MPU_rot['mean_gyro_X'][i] = ax_e
            mean_list_MPU_rot['mean_gyro_Y'][i] = ay_e
            mean_list_MPU_rot['mean_gyro_Z'][i] = az_e
        return error_sum / meas_len


    x_0 = [0, 0, 0]
    minimize(mpu_find_rotation, np.array(x_0), method='L-BFGS-B',
             # bounds=[(0.9, 1.1), (0.9, 1.1), (0.9, 1.1), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)],
             bounds=[(-360, 360), (-360, 360), (-360, 360)],
             options={'maxfun': 100000, 'eps': 0.0000001, 'ftol': 0.000001, 'iprint': -1})
    print('Rotation angles for MPU sensor: ', MPU_fin_rot)

    if True:
        plt.style.use('dark_background')

        ax1 = plt.figure().add_subplot(projection='3d')
        ax1.set_xlabel('acceleration X')
        ax1.set_ylabel('acceleration Y')
        ax1.set_zlabel('acceleration Z')
        ax1.scatter(mean_list_ISM_raw['mean_acc_X'], mean_list_ISM_raw['mean_acc_Y'], mean_list_ISM_raw['mean_acc_Z'],
                    label='ISM_sensor_raw_acc', marker='o')
        ax1.scatter(mean_list_MPU_raw['mean_acc_X'], mean_list_MPU_raw['mean_acc_Y'], mean_list_MPU_raw['mean_acc_Z'],
                    label='MPU_sensor_raw_acc', marker='^')

        ax2 = plt.figure().add_subplot(projection='3d')
        ax2.set_xlabel('acceleration X')
        ax2.set_ylabel('acceleration Y')
        ax2.set_zlabel('acceleration Z')
        ax2.scatter(mean_list_ISM_target['mean_acc_X'], mean_list_ISM_target['mean_acc_Y'],
                    mean_list_ISM_target['mean_acc_Z'], label='Target_acc', marker='o')
        ax2.scatter(mean_list_ISM_rot['mean_acc_X'], mean_list_ISM_rot['mean_acc_Y'],
                    mean_list_ISM_rot['mean_acc_Z'], label='Final ISM', marker='^')

        ax3 = plt.figure().add_subplot(projection='3d')
        ax3.set_xlabel('acceleration X')
        ax3.set_ylabel('acceleration Y')
        ax3.set_zlabel('acceleration Z')
        ax3.scatter(mean_list_ISM_target['mean_acc_X'], mean_list_ISM_target['mean_acc_Y'],
                    mean_list_ISM_target['mean_acc_Z'], label='Target_acc', marker='o')
        ax3.scatter(mean_list_MPU_rot['mean_acc_X'], mean_list_MPU_rot['mean_acc_Y'],
                    mean_list_MPU_rot['mean_acc_Z'], label='Final MPU', marker='^')

        plt.show()
