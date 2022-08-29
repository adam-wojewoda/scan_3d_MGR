from math import sqrt
import pandas as pd


class SensorCurve3D:
    def __init__(self):
        # location
        self.x_point_list = []
        self.y_point_list = []
        self.z_point_list = []
        # speed
        self.x_speed_list = []
        self.y_speed_list = []
        self.z_speed_list = []
        # acceleration
        self.x_acc_list = []
        self.y_acc_list = []
        self.z_acc_list = []
        # time_table
        self.time_list = []
        self.dt_step = 0.0
        self.time_last = 0.0
        self.x_point_last = 0.0
        self.y_point_last = 0.0
        self.z_point_last = 0.0
        self.x_speed_last = 0.0
        self.y_speed_last = 0.0
        self.z_speed_last = 0.0
        self.x_acc_last = 0.0
        self.y_acc_last = 0.0
        self.z_acc_last = 0.0
        self.g = 9.8159
        self.acc_last = self.g
        # self.index = 0
        self.corrector = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def set_input_tables(self, x_acc, y_acc, z_acc, times):
        # print('start')
        self.x_acc_list = x_acc[:]
        # print('x_ok')
        self.y_acc_list = y_acc[:]
        # print('y_ok')
        self.z_acc_list = z_acc[:]
        # print('z_ok')
        self.time_list = times[:]
        # print('t_ok')
        pass

    def fill_curve_from_input_tables(self, g_in, corrections):
        if corrections is not None:
            self.corrector = corrections
        l = [self.time_list[:], self.x_acc_list[:], self.y_acc_list[:], self.z_acc_list[:]]
        input_tables = list(map(list, zip(*l)))
        self.x_point_list = []
        self.y_point_list = []
        self.z_point_list = []
        self.x_speed_list = []
        self.y_speed_list = []
        self.z_speed_list = []
        self.x_point_last = 0.0
        self.y_point_last = 0.0
        self.z_point_last = 0.0
        self.x_speed_last = 0.0
        self.y_speed_last = 0.0
        self.z_speed_last = 0.0
        self.x_acc_last = self.x_acc_list[0]
        self.y_acc_last = self.y_acc_list[0]
        self.z_acc_last = self.z_acc_list[0]
        self.time_last = self.time_list[0]
        self.g = g_in
        self.acc_last = self.g

        for row in input_tables:
            self.dt_step = row[0] - self.time_last
            self.time_last = row[0]

            # estimate_new_speeds
            x_speed_now = self.x_speed_last + self.dt_step * (row[1] + self.x_acc_last) * 0.5 - self.corrector['x']
            y_speed_now = self.y_speed_last + self.dt_step * (row[2] + self.y_acc_last) * 0.5 - self.corrector['y']
            z_speed_now = self.z_speed_last + self.dt_step * ((row[3] + self.z_acc_last) * 0.5 - self.g) - \
                          self.corrector['z']
            self.x_acc_last = row[1]
            self.y_acc_last = row[2]
            self.z_acc_last = row[3]
            self.x_point_last = self.x_point_last + self.dt_step * (self.x_speed_last + x_speed_now) * 0.5
            self.y_point_last = self.y_point_last + self.dt_step * (self.y_speed_last + y_speed_now) * 0.5
            self.z_point_last = self.z_point_last + self.dt_step * (self.z_speed_last + z_speed_now) * 0.5
            self.x_speed_last = x_speed_now
            self.y_speed_last = y_speed_now
            self.z_speed_last = z_speed_now
            # self.acc_last = sqrt(row[1] ** 2 + row[2] ** 2 + row[3] ** 2)
            # add new points to lists
            self.x_point_list.append(self.x_point_last)
            self.y_point_list.append(self.y_point_last)
            self.z_point_list.append(self.z_point_last)
            self.x_speed_list.append(self.x_speed_last)
            self.y_speed_list.append(self.y_speed_last)
            self.z_speed_list.append(self.z_speed_last)
            pass

    def fill_curve(self, df_in, g_in, corrections):
        self.dt_step = 0.0
        self.x_point_list = []
        self.y_point_list = []
        self.z_point_list = []
        self.x_speed_list = []
        self.y_speed_list = []
        self.z_speed_list = []
        self.time_list = []
        self.x_point_last = 0.0
        self.y_point_last = 0.0
        self.z_point_last = 0.0
        self.x_speed_last = 0.0
        self.y_speed_last = 0.0
        self.z_speed_last = 0.0
        self.g = g_in
        self.acc_last = self.g
        if corrections is not None:
            self.corrector = corrections
        self.time_last = df_in['mean_time_rel'].iloc[0]

        # self.index = 0

        def add_point(row):
            self.dt_step = row['mean_time_rel'] - self.time_last
            self.time_last = row['mean_time_rel']
            # estimate_new_points
            x_speed_now = self.x_speed_last + self.dt_step * (row['acc_X_e'] + self.x_acc_last) * 0.5 - self.corrector[
                'x']
            y_speed_now = self.y_speed_last + self.dt_step * (row['acc_Y_e'] + self.y_acc_last) * 0.5 - self.corrector[
                'y']
            z_speed_now = self.z_speed_last + self.dt_step * ((row['acc_Z_e'] + self.z_acc_last) * 0.5 - self.g) - \
                          self.corrector['z']
            self.x_acc_last = row['acc_X_e']
            self.y_acc_last = row['acc_Y_e']
            self.z_acc_last = row['acc_Z_e']
            self.x_point_last = self.x_point_last + self.dt_step * (self.x_speed_last + x_speed_now) * 0.5
            self.y_point_last = self.y_point_last + self.dt_step * (self.y_speed_last + y_speed_now) * 0.5
            self.z_point_last = self.z_point_last + self.dt_step * (self.z_speed_last + z_speed_now) * 0.5
            self.x_speed_last = x_speed_now
            self.y_speed_last = y_speed_now
            self.z_speed_last = z_speed_now

            # self.x_point_last = self.x_point_last + self.dt_step * self.x_speed_last
            # self.y_point_last = self.y_point_last + self.dt_step * self.y_speed_last
            # self.z_point_last = self.z_point_last + self.dt_step * self.z_speed_last
            #
            # # estimate_new_speeds
            # self.x_speed_last = self.x_speed_last + self.dt_step * row['acc_X_e'] - self.corrector['x']
            # self.y_speed_last = self.y_speed_last + self.dt_step * row['acc_Y_e'] - self.corrector['y']
            # self.z_speed_last = self.z_speed_last + self.dt_step * (row['acc_Z_e'] - self.g) - self.corrector['z']
            # self.acc_last = sqrt(row['acc_X_e'] ** 2 + row['acc_Y_e'] ** 2 + row['acc_Z_e'] ** 2)
            # add new points to lists
            self.x_point_list.append(self.x_point_last)
            self.y_point_list.append(self.y_point_last)
            self.z_point_list.append(self.z_point_last)
            self.x_speed_list.append(self.x_speed_last)
            self.y_speed_list.append(self.y_speed_last)
            self.z_speed_list.append(self.z_speed_last)
            self.time_list.append(row['mean_time_rel'])

        df_in.apply(lambda row: add_point(row), axis=1)

    def return_as_df(self):
        lis = [self.time_list[:], self.x_point_list[:], self.y_point_list[:], self.z_point_list[:]]
        input_tables = list(map(list, zip(*lis)))
        df_temp = pd.DataFrame(input_tables, columns=['mean_time_rel', 'X_p_e', 'Y_p_e', 'Z_p_e'])
        df_temp.set_index('mean_time_rel', inplace=True)
        return df_temp
