from math import sqrt, nan, atan2
from scipy.spatial.transform import Rotation


def rotate_acceleration_row(row, acc_names=None, degrees=True):
    # df_in['phi_hat'] = np.nan
    # df_in['psi_hat'] = np.nan
    # df_in['theta_hat'] = np.nan
    if acc_names is None:
        acc_names = ['X_acc', 'Y_acc', 'Z_acc']
    rot = Rotation.from_euler('ZYX', [row['psi'], row['theta'], row['phi']], degrees=degrees)
    return rot.apply([row[acc_names[0]], row[acc_names[1]], row[acc_names[2]]], inverse=False)


def rotate_vect_row(row):
    vect_names = ['X', 'Y', 'Z']
    ang_names = ['psi_hat', 'theta_hat', 'phi_hat']
    # df_in['phi_hat'] = np.nan
    # df_in['psi_hat'] = np.nan
    # df_in['theta_hat'] = np.nan
    rot = Rotation.from_euler('ZYX', [row[ang_names[0]], row[ang_names[1]], row[ang_names[2]]], degrees=False)
    [ax_e, ay_e, az_e] = rot.apply([row[vect_names[0]], row[vect_names[1]], row[vect_names[2]]], inverse=False)
    row[vect_names[0] + '_e'] = ax_e
    row[vect_names[1] + '_e'] = ay_e
    row[vect_names[2] + '_e'] = az_e
    # row['g_ground'] = sqrt(ax_e ** 2.0 + ay_e ** 2.0 + az_e ** 2)
    return row


def rotate_acc(data_in):
    # rotate acceleration
    acc_earth_list = []
    for row in data_in:
        axe, aye, aze = rotate_acceleration_row(row)
        acc_earth_list.append({'X_acc_e': axe,
                               'Y_acc_e': aye,
                               'Z_acc_e': aze})
    return acc_earth_list


def rotate_vect(df_in):
    vect_names = ['X', 'Y', 'Z']
    df_tmp = df_in.copy()
    print('df_copied')
    df_tmp[vect_names[0] + '_e'] = nan
    df_tmp[vect_names[1] + '_e'] = nan
    df_tmp[vect_names[2] + '_e'] = nan
    # df_in['g_ground'] = nan
    df_tmp.apply(lambda row: rotate_vect_row(row), axis=1)
    return df_tmp[[vect_names[0] + '_e', vect_names[1] + '_e', vect_names[2] + '_e']]


def get_acc_angles(acc_x, acc_y, acc_z):
    acc_phi = atan2(acc_y, sqrt(acc_x ** 2.0 + acc_z ** 2.0))
    acc_theta = atan2(-acc_x, sqrt(acc_y ** 2.0 + acc_z ** 2.0))
    return acc_phi, acc_theta
