from math import sqrt, nan, atan2
from scipy.spatial.transform import Rotation


class AngleCalculator:
    """Class holds all the functions considering getting angles from acc data"""

    @staticmethod
    def rotate_acceleration_row(row, acc_names=['mean_acc_X_t', 'mean_acc_Y_t', 'mean_acc_Z_t']):
        # df_in['phi_hat'] = np.nan
        # df_in['psi_hat'] = np.nan
        # df_in['theta_hat'] = np.nan
        rot = Rotation.from_euler('ZYX', [row['psi_hat'], row['theta_hat'], row['phi_hat']], degrees=False)
        [ax_e, ay_e, az_e] = rot.apply([row[acc_names[0]], row[acc_names[1]], row[acc_names[2]]], inverse=False)
        row['acc_X_e'] = ax_e
        row['acc_Y_e'] = ay_e
        row['acc_Z_e'] = az_e
        row['g_ground'] = sqrt(ax_e ** 2.0 + ay_e ** 2.0 + az_e ** 2)
        return row

    @staticmethod
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

    def rotate_acc(self, df_in):
        df_in['acc_X_e'] = nan
        df_in['acc_Y_e'] = nan
        df_in['acc_Z_e'] = nan
        df_in['g_ground'] = nan
        df_in.apply(lambda row: self.rotate_acceleration_row(row), axis=1)
        return df_in

    def rotate_vect(self, df_in):
        vect_names = ['X', 'Y', 'Z']
        df_tmp=df_in.copy()
        print('df_copied')
        df_tmp[vect_names[0] + '_e'] = nan
        df_tmp[vect_names[1] + '_e'] = nan
        df_tmp[vect_names[2] + '_e'] = nan
        #df_in['g_ground'] = nan
        df_tmp.apply(lambda row: self.rotate_vect_row(row), axis=1)
        return df_tmp[[vect_names[0] + '_e',vect_names[1] + '_e',vect_names[2] + '_e']]

    @staticmethod
    def get_acc_angles(row, acc_names=['mean_acc_X_t', 'mean_acc_Y_t', 'mean_acc_Z_t']):
        row['acc_phi'] = atan2(row[acc_names[1]],
                               sqrt(row[acc_names[0]] ** 2.0 + row[acc_names[2]] ** 2.0))
        row['acc_theta'] = atan2(-row[acc_names[0]],
                                 sqrt(row[acc_names[1]] ** 2.0 + row[[acc_names[2]]] ** 2.0))
        return row

    def calculate_angles_1(self, df_in):
        df_in['acc_phi'] = nan
        df_in['acc_theta'] = nan
        df_in.apply(lambda row: self.get_acc_angles(row), axis=1)
        return df_in
