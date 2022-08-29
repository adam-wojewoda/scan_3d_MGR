import pandas as pd
from numpy import array, cross, pi
from scipy.spatial.transform import Rotation


class GeometryConstants:
    # Defining zero point (this is root of device frame) (actually not needed)
    ZERO_POINT = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'unit': 'm'}

    # Position of ISM sensor (in device frame)
    ISM_330_POS = {'X': 0.101, 'Y': 0.101, 'Z': 0.101, 'unit': 'm'}

    # Rotation of ISM sensor (in reference to device frame)
    ISM_330_ROT = {'X': 0.0, 'Y': 90.0, 'Z': 0.0, 'order': 'ZYX', 'unit': 'deg'}

    # Position of MPU sensor (in device frame)
    MPU_9255_POS = {'X': 0.202, 'Y': 0.202, 'Z': 0.202, 'unit': 'm'}

    # Rotation of MPU sensor (in reference to device frame)
    MPU_9255_ROT = {'X': 0.0, 'Y': 135.0, 'Z': -135.0, 'order': 'XYZ', 'unit': 'deg'}

    # Position of scanner zero (in device frame)
    SCANNER_POS = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'unit': 'm'}

    # Rotation of scanner zero (in device frame)
    SCANNER_ROT = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'order': 'XYZ', 'unit': 'deg'}


class GeometryVectors(GeometryConstants):
    def __init__(self):
        self.position_vectors = {'ISM_330': array([self.ISM_330_POS['X'] - self.ZERO_POINT['X'],
                                                   self.ISM_330_POS['Y'] - self.ZERO_POINT['Y'],
                                                   self.ISM_330_POS['Z'] - self.ZERO_POINT['Z']]),

                                 'MPU_9255': array([self.MPU_9255_POS['X'] - self.ZERO_POINT['X'],
                                                    self.MPU_9255_POS['Y'] - self.ZERO_POINT['Y'],
                                                    self.MPU_9255_POS['Z'] - self.ZERO_POINT['Z']]),
                                 'SCANNER': array([self.SCANNER_POS['X'] - self.ZERO_POINT['X'],
                                                   self.SCANNER_POS['Y'] - self.ZERO_POINT['Y'],
                                                   self.SCANNER_POS['Z'] - self.ZERO_POINT['Z']])
                                 }
        self.rotations = {
            'ISM_330': {'vect': array([self.ISM_330_ROT['X'], self.ISM_330_ROT['Y'], self.ISM_330_ROT['Z']]),
                        'order': self.ISM_330_ROT['order'],
                        'degrees': self.ISM_330_ROT['unit'] == 'deg'},
            'MPU_9255': {'vect': array([self.MPU_9255_ROT['X'], self.MPU_9255_ROT['Y'], self.MPU_9255_ROT['Z']]),
                         'order': self.MPU_9255_ROT['order'],
                         'degrees': self.MPU_9255_ROT['unit'] == 'deg'},
            'SCANNER': {'vect': array([self.SCANNER_ROT['X'], self.SCANNER_ROT['Y'], self.SCANNER_ROT['Z']]),
                        'order': self.SCANNER_ROT['order'],
                        'degrees': self.SCANNER_ROT['unit'] == 'deg'}}

    def rotation_apply_vect(self, df_in, names, rotation_info1):
        df_out = df_in.copy()
        rot = Rotation.from_euler(self.rotations[rotation_info1]['order'], self.rotations[rotation_info1]['vect'],
                                  degrees=self.rotations[rotation_info1]['degrees'])
        df_out[names] = rot.apply(df_in[names], inverse=False)

        return df_out

    @staticmethod
    def translate_acceleration_row(row, acc_names, gyro_names, gyro_der_names, radius_array, output_names, ang_unit_mult = 1.0):
        # a_A=a_B + w_der x r_A/B + w x (w x r_A/B))

        a_b = array([row[acc_names[0]], row[acc_names[1]], row[acc_names[2]]])
        w_der = array([row[gyro_der_names[0]], row[gyro_der_names[1]], row[gyro_der_names[2]]])*ang_unit_mult
        w = array([row[gyro_names[0]], row[gyro_names[1]], row[gyro_names[2]]]) * ang_unit_mult

        a_a = a_b + cross(w_der, radius_array) + cross(w, cross(w, radius_array))
        row[output_names[0]] = a_a[0] / ang_unit_mult
        row[output_names[1]] = a_a[1] / ang_unit_mult
        row[output_names[2]] = a_a[2] / ang_unit_mult
        return row

    def translate_acceleration_vect(self, df_in, sensor, acc_names, gyro_names, gyro_der_names, output_names, ang_unit = 'deg'):
        df_out = df_in.copy()

        if ang_unit == 'deg':
            mult = pi/180
        elif ang_unit == 'rad':
            mult = 1.0
        else:
            raise ValueError('Wrong unit name use "deg" or "rad"!')

        position = self.position_vectors[sensor]
        a_b_0 = df_in[acc_names].to_numpy()
        a_b = [{'a_b':x} for x in a_b_0]

        w_der_0 = df_in[gyro_der_names].to_numpy() * mult
        w_der = [{'w_der':x} for x in w_der_0]

        w_0 = df_in[gyro_names].to_numpy() * mult
        w = [{'w': x} for x in w_0]
        arr_tmp = array([a_b,w_der,w])
        arr_trans = [ {**row[0],**row[1],**row[2]} for row in arr_tmp.transpose().tolist()]

        a_a = array([row['a_b'] + cross(row['w_der'], position) + cross(row['w'], cross(row['w'], position))  for row in arr_trans]).transpose()

        df_out[output_names[0]] = a_a[0]
        df_out[output_names[1]] = a_a[1]
        df_out[output_names[2]] = a_a[2]

        return df_out

    def translate_acceleration(self, df_in, sensor, acc_names, gyro_names, gyro_der_names, output_names, ang_unit = 'deg'):
        df_out = df_in.copy()

        if ang_unit == 'deg':
            df_out.apply(
                lambda row: self.translate_acceleration_row(row, acc_names, gyro_names, gyro_der_names,
                                                            self.position_vectors[sensor],
                                                            output_names, ang_unit_mult=pi/180), axis=1)
        elif ang_unit == 'rad':
            df_out.apply(
                lambda row: self.translate_acceleration_row(row, acc_names, gyro_names, gyro_der_names,
                                                            self.position_vectors[sensor],
                                                            output_names, ang_unit_mult=1.0), axis=1)
        else:
            raise ValueError('Wrong unit name use "deg" or "rad"!')

        # df_out.drop(df_out.columns.difference(gyro_names + output_names), 1, inplace=True)
        return df_out

if __name__ == '__main__':
    data_rot_ISM=pd.read_csv('/media/adamw/DATA/Projekty/Praca_mgr/scan_3d_MGR/gui_server/ism_test.csv')
    device_geometry = GeometryVectors()
    #print(data_rot_ISM.describe())
    data_trans_ISM = device_geometry.translate_acceleration_vect(df_in=data_rot_ISM,
                                                                 sensor='ISM_330',
                                                                 acc_names=['mean_acc_X_b', 'mean_acc_Y_b',
                                                                            'mean_acc_Z_b'],
                                                                 gyro_names=['mean_gyro_X_b', 'mean_gyro_Y_b',
                                                                             'mean_gyro_Z_b'],
                                                                 gyro_der_names=['mean_gyro_X_b_der',
                                                                                 'mean_gyro_Y_b_der',
                                                                                 'mean_gyro_Z_b_der'],
                                                                 output_names=['mean_acc_X_t', 'mean_acc_Y_t',
                                                                               'mean_acc_Z_t'],
                                                                 ang_unit='deg')
    pd.set_option('display.max_columns', None)
    print(data_trans_ISM.tail())