from numpy import nan


class BiasCompensator:
    def __init__(self, sensor='', mode='basic'):
        self.mode = mode
        if sensor == 'MPU_9255':
            self.sensor_type = sensor
            self.bias_const = {
                'x_gyro': 0.0, 'y_gyro': 0.0, 'z_gyro': 0.0,
                'x_acc': 0.11933902, 'y_acc': 0.07924577, 'z_acc': 0.36710142}
            self.bias_linear = {
                # 'a_x_gyro': 1.0, 'a_y_gyro': 1.0, 'a_z_gyro': 1.0,
                'a_x_acc': 1.00144885, 'a_y_acc': 1.00345854, 'a_z_acc': 0.98710848,
                # 'b_x_gyro': 0.0, 'b_y_gyro': 0.0, 'b_z_gyro': 0.0,
                'b_x_acc': 0.05409835, 'b_y_acc': -0.04451797, 'b_z_acc': 0.35282425}
        elif sensor == 'ISM_330':
            self.sensor_type = sensor
            self.bias_const = {
                'x_gyro': 0.0, 'y_gyro': 0.0, 'z_gyro': 0.0,
                'x_acc': -0.00885079, 'y_acc': -0.02427085, 'z_acc': -0.10525718}
            self.bias_linear = {
                # 'a_x_gyro': 1.0, 'a_y_gyro': 1.0, 'a_z_gyro': 1.0,
                'a_x_acc': 0.99672208, 'a_y_acc': 0.99807871, 'a_z_acc': 0.99786917,
                # 'b_x_gyro': 0.0, 'b_y_gyro': 0.0, 'b_z_gyro': 0.0,
                'b_x_acc': -0.04897471, 'b_y_acc': 0.1070396, 'b_z_acc': -0.07113173}
        else:
            self.sensor_type = None
            raise ValueError('Sensor not recognised!')

        if self.mode == 'basic':
            self.apply_bias = self.apply_bias_basic_vect
        elif self.mode == 'linear':
            self.apply_bias = self.apply_bias_linear_vect
        else:
            raise ValueError('Method not recognised!')

    def change_bias_acc_const(self, x_in, y_in, z_in):
        self.bias_const['x_acc'] = x_in
        self.bias_const['y_acc'] = y_in
        self.bias_const['z_acc'] = z_in

    def change_bias_acc_linear(self, a_x_in, a_y_in, a_z_in, b_x_in, b_y_in, b_z_in):
        self.bias_linear['a_x_acc'] = a_x_in
        self.bias_linear['a_y_acc'] = a_y_in
        self.bias_linear['a_z_acc'] = a_z_in
        self.bias_linear['b_x_acc'] = b_x_in
        self.bias_linear['b_y_acc'] = b_y_in
        self.bias_linear['b_z_acc'] = b_z_in

    def change_bias_gyro_const(self, x_in, y_in, z_in):
        self.bias_const['x_gyro'] = x_in
        self.bias_const['y_gyro'] = y_in
        self.bias_const['z_gyro'] = z_in
        # self.bias_linear['b_x_gyro'] = self.bias_const['x_gyro']
        # self.bias_linear['b_y_gyro'] = self.bias_const['y_gyro']
        # self.bias_linear['b_z_gyro'] = self.bias_const['z_gyro']

    def set_gyro_const_bias(self, n_rows, df_in):
        self.bias_const['x_gyro'] = -df_in.iloc[:n_rows]['mean_gyro_X'].mean()
        self.bias_const['y_gyro'] = -df_in.iloc[:n_rows]['mean_gyro_Y'].mean()
        self.bias_const['z_gyro'] = -df_in.iloc[:n_rows]['mean_gyro_Z'].mean()

    #    self.bias_linear['b_x_gyro'] = self.bias_const['x_gyro']
    #    self.bias_linear['b_y_gyro'] = self.bias_const['y_gyro']
    #    self.bias_linear['b_z_gyro'] = self.bias_const['z_gyro']
    # @staticmethod
    # def add_const_bias(row, bias_array):
    #     row['mean_acc_X_b'] = row['mean_acc_X'] + bias_array['x_acc']
    #     row['mean_acc_Y_b'] = row['mean_acc_Y'] + bias_array['y_acc']
    #     row['mean_acc_Z_b'] = row['mean_acc_Z'] + bias_array['z_acc']
    #     row['mean_gyro_X_b'] = row['mean_gyro_X'] + bias_array['x_gyro']
    #     row['mean_gyro_Y_b'] = row['mean_gyro_Y'] + bias_array['y_gyro']
    #     row['mean_gyro_Z_b'] = row['mean_gyro_Z'] + bias_array['z_gyro']
    #     return row

    # @staticmethod
    # def add_const_bias_vect(column_in, b):
    #     # adds linear bias to single column
    #     return column_in + b

    # def apply_bias_basic(self, df_in):
    #     # df_in['mean_acc_X_b'] = nan
    #     # df_in['mean_acc_Y_b'] = nan
    #     # df_in['mean_acc_Z_b'] = nan
    #     # df_in['mean_gyro_X_b'] = nan
    #     # df_in['mean_gyro_Y_b'] = nan
    #     # df_in['mean_gyro_Y_b'] = nan
    #     return df_in.apply(lambda row: self.add_const_bias(row, self.bias_const.copy()), axis=1)
    #     # return df_in.apply(add_const_bias, axis=1)

    def apply_bias_basic_vect(self, df_in):
        df_out = df_in.copy()
        df_out['mean_acc_X_b'] = df_in['mean_acc_X'] + self.bias_const['x_acc']
        df_out['mean_acc_Y_b'] = df_in['mean_acc_Y'] + self.bias_const['y_acc']
        df_out['mean_acc_Z_b'] = df_in['mean_acc_Z'] + self.bias_const['z_acc']
        df_out['mean_gyro_X_b'] = df_in['mean_gyro_X'] + self.bias_const['x_gyro']
        df_out['mean_gyro_Y_b'] = df_in['mean_gyro_Y'] + self.bias_const['y_gyro']
        df_out['mean_gyro_Z_b'] = df_in['mean_gyro_Z'] + self.bias_const['z_gyro']
        return df_out
        # return df_in.apply(add_const_bias, axis=1)

    # @staticmethod
    # def add_linear_bias(row, bias_linear, bias_const):
    #     row['mean_acc_X_b'] = row['mean_acc_X'] * bias_linear['a_x_acc'] + bias_linear['b_x_acc']
    #     row['mean_acc_Y_b'] = row['mean_acc_Y'] * bias_linear['a_y_acc'] + bias_linear['b_y_acc']
    #     row['mean_acc_Z_b'] = row['mean_acc_Z'] * bias_linear['a_z_acc'] + bias_linear['b_z_acc']
    #     row['mean_gyro_X_b'] = row['mean_gyro_X'] + bias_const['x_gyro']
    #     row['mean_gyro_Y_b'] = row['mean_gyro_Y'] + bias_const['y_gyro']
    #     row['mean_gyro_Z_b'] = row['mean_gyro_Z'] + bias_const['z_gyro']
    #     return row

    # @staticmethod
    # def add_linear_bias_vect(column_in, a, b):
    #     # adds linear bias to single column
    #     return column_in * a + b

    # def apply_bias_linear(self, df_in):
    #     return df_in.apply(lambda row: self.add_linear_bias(row, self.bias_linear, self.bias_const), axis=1)
    #     # return df_in.apply(add_linear_bias, axis=1)

    def apply_bias_linear_vect(self, df_in):
        df_out = df_in.copy()
        df_out['mean_acc_X_b'] = df_in['mean_acc_X'] * self.bias_linear['a_x_acc'] + self.bias_linear['b_x_acc']
        df_out['mean_acc_Y_b'] = df_in['mean_acc_Y'] * self.bias_linear['a_y_acc'] + self.bias_linear['b_y_acc']
        df_out['mean_acc_Z_b'] = df_in['mean_acc_Z'] * self.bias_linear['a_z_acc'] + self.bias_linear['b_z_acc']
        df_out['mean_gyro_X_b'] = df_in['mean_gyro_X'] + self.bias_const['x_gyro']
        df_out['mean_gyro_Y_b'] = df_in['mean_gyro_Y'] + self.bias_const['y_gyro']
        df_out['mean_gyro_Z_b'] = df_in['mean_gyro_Z'] + self.bias_const['z_gyro']
        return df_out
