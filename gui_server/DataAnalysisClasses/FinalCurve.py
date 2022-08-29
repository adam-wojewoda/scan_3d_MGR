from __future__ import annotations

from typing import Dict

from scipy.spatial.transform import Rotation
from numpy import nan

total_nodes = 0
total_points = 0


class Voxel:  # Voxel structure
    def __init__(self, min_points, min_dimension):
        self.minimal_points_number = min_points
        self.minimal_voxel_dimension = min_dimension
        self.points: list[ScannerPoint] = []  # list of points references
        self.mean_plane = None  # mean voxel plane
        self.sub_voxels: list[Voxel] = []  # list of sub_voxels
        self.range: Dict[str, float] = {'x_min': nan, 'x_max': nan,  # <x_min;x_max)
                                        'y_min': nan, 'y_max': nan,  # <y_min;y_max)
                                        'z_min': nan, 'z_max': nan}  # <z_min;z_max)

    def set_range_from_points(self):
        if self.points:
            range_temp = {'x_min': self.points[0].pos_glob_vect['X'], 'x_max': self.points[0].pos_glob_vect['X'],
                          # <x_min;x_max)
                          'y_min': self.points[0].pos_glob_vect['Y'], 'y_max': self.points[0].pos_glob_vect['Y'],
                          # <y_min;y_max)
                          'z_min': self.points[0].pos_glob_vect['Z'], 'z_max': self.points[0].pos_glob_vect['Z']
                          # <z_min;z_max)
                          }

            for point in self.points:
                # X
                if point.pos_glob_vect['X'] < range_temp['x_min']:
                    range_temp['x_min'] = point.pos_glob_vect['X']
                else:
                    if point.pos_glob_vect['X'] > range_temp['x_max']:
                        range_temp['x_max'] = point.pos_glob_vect['X']
                # Y
                if point.pos_glob_vect['Y'] < range_temp['y_min']:
                    range_temp['y_min'] = point.pos_glob_vect['Y']
                else:
                    if point.pos_glob_vect['Y'] > range_temp['y_max']:
                        range_temp['y_max'] = point.pos_glob_vect['Y']
                # Z
                if point.pos_glob_vect['Z'] < range_temp['z_min']:
                    range_temp['z_min'] = point.pos_glob_vect['Z']
                else:
                    if point.pos_glob_vect['Z'] > range_temp['z_max']:
                        range_temp['z_max'] = point.pos_glob_vect['Z']

            self.range = range_temp
        else:
            raise RuntimeError('Empty list of points')

    def fill_root_voxel(self, measurements_in: list[MeasurementInstance]):
        for meas in measurements_in:
            self.points.extend(meas.scanner_points)

        print('Points to Voxelize: ', len(self.points))
        self.set_range_from_points()

    def divide_voxel(self):
        # Choose splitting axis
        # Check for None
        for key, val in self.range.items():
            if val is nan:
                raise RuntimeError('No or wrong ranges set')

        dx = self.range['x_max'] - self.range['x_min']
        dy = self.range['y_max'] - self.range['y_min']
        dz = self.range['z_max'] - self.range['z_min']

        d_max = max(dx, dy, dz)

        if d_max > self.minimal_voxel_dimension and len(self.points) > self.minimal_points_number:
            range_tmp_1 = self.range.copy()
            range_tmp_2 = self.range.copy()
            # border = nan
            # axis = ''
            if d_max == dx:
                # subdivide by X
                axis = 'X'
                border = range_tmp_1['x_min'] + (d_max / 2)
                range_tmp_1['x_max'] = border
                range_tmp_2['x_min'] = border
                pass
            else:
                if d_max == dy:
                    # subdivide by Y
                    axis = 'Y'
                    border = range_tmp_1['y_min'] + (d_max / 2)
                    range_tmp_1['y_max'] = border
                    range_tmp_2['y_min'] = border
                    pass
                else:
                    if d_max == dz:
                        # subdivide by Z
                        axis = 'Z'
                        border = range_tmp_1['z_min'] + (d_max / 2)
                        range_tmp_1['z_max'] = border
                        range_tmp_2['z_min'] = border
                        pass
                    else:
                        raise RuntimeError('Improper voxel range values')

            # Create two empty voxels and set their ranges
            v1 = Voxel(self.minimal_points_number, self.minimal_voxel_dimension)
            v1.range = range_tmp_1
            v2 = Voxel(self.minimal_points_number, self.minimal_voxel_dimension)
            v2.range = range_tmp_2

            # Divide current voxel points between two new points
            while self.points:
                point: ScannerPoint = self.points.pop()
                if point.pos_glob_vect[axis] < border:
                    v1.points.append(point)
                else:
                    v2.points.append(point)

            if v1.points:
                self.sub_voxels.append(v1)
            if v2.points:
                self.sub_voxels.append(v2)
            for v in self.sub_voxels:
                v.divide_voxel()

    def drop_unneeded_nodes(self):
        # If my voxel has only one sub-voxel containing no points, then I can replace these two sub-voxels with one
        # In other words EAT USELESS CHILDREN and keep the grandkids
        end = False
        while not end:
            if self.points:
                end = True
                return
            else:
                if len(self.sub_voxels) == 1:
                    if self.sub_voxels[0].points:
                        end = True
                        return
                    else:
                        # steal grandchildren
                        self.sub_voxels: list[Voxel] = self.sub_voxels[0].sub_voxels

                else:
                    for v in self.sub_voxels:
                        v.drop_unneeded_nodes()
                        end = True
                        return

    def describe_tree(self, level_in=1, code_in='root'):
        global total_nodes
        total_nodes = total_nodes + 1
        print('  ')
        print('Level: ', str(level_in))
        print('Tree_code: ', code_in)
        print('Sub-nodes: ', str(len(self.sub_voxels)))
        if self.points:
            print('Points: ', str(len(self.points)))
        else:
            i = 0
            for v in self.sub_voxels:
                v.describe_tree(level_in + 1, code_in + str(i))
                i = i + 1

    def count_nodes(self):
        global total_nodes
        total_nodes = total_nodes + 1
        if self.points:
            pass
        else:
            for v in self.sub_voxels:
                v.count_nodes()

    def count_points(self):
        global total_points
        if self.points:
            total_points = total_points + len(self.points)
        else:
            for v in self.sub_voxels:
                v.count_points()


class ScannerPoint:  # Single scanner point
    def __init__(self, point_in):
        self.voxel_reference = None
        self.pos_glob_vect = {'X': nan, 'Y': nan, 'Z': nan}  # global point position (earth frame)
        self.pos_rel_vect = {'X': nan, 'Y': nan, 'Z': nan}  # relative point position (earth frame)
        # self.pos_dev_vect = {'X': None, 'Y': None, 'Z': None}  # device frame point position (device frame)
        self.pos_dev_vect = point_in.copy()
        self.opt_vect = {'X': nan, 'Y': nan, 'Z': nan}  # optimization force vector

    def calculate_relative_vect(self, rot_in: Rotation):
        tmp = rot_in.apply([self.pos_dev_vect['X'], self.pos_dev_vect['Y'], self.pos_dev_vect['Z']], inverse=False)
        self.pos_rel_vect = {'X': tmp[0], 'Y': tmp[1], 'Z': tmp[2]}

    def calculate_global_vect(self, dev_global_pos):
        self.pos_glob_vect = {k: self.pos_rel_vect.get(k, 0) + dev_global_pos.get(k, 0) for k in
                              set(self.pos_rel_vect) & set(dev_global_pos)}


class OptVectorDev:  # Optimization force vector structure
    def __init__(self):
        self.pos_loc_vect = {'X': nan, 'Y': nan, 'Z': nan}  # local needed position change 'force'
        self.ang_loc_vect = {'X': nan, 'Y': nan, 'Z': nan}  # local needed rotation change 'force' (rot_vect)
        self.pos_glob_vect = {'X': nan, 'Y': nan, 'Z': nan}  # global needed position change 'force'
        self.ang_glob_vect = {'X': nan, 'Y': nan, 'Z': nan}  # global needed rotation change 'force' (rot_vect)


class DevicePosition:  # Everything regarding device position in space + optimization vector instance
    def __init__(self, mean_time_rel, dev_pos, dev_rot_vect):
        self.mean_time_rel = mean_time_rel
        self.time_delta = nan  # time from last position
        self.pos_delta = {'X': nan, 'Y': nan, 'Z': nan}  # translation from last point in [m]
        self.rot_delta = None
        # self.pos_abs = {'X': None, 'Y': None, 'Z': None}  # absolute linear position [m]
        self.pos_abs = dev_pos.copy()
        # self.rot_abs = {'X': None, 'Y': None, 'Z': None}  # abs. scanner rot. vect [rad] (rot. of coordinate system)
        self.rot_abs = Rotation.from_rotvec([dev_rot_vect['X'], dev_rot_vect['Y'], dev_rot_vect['Z']])
        self.opt_vect = OptVectorDev()

    def set_position_deltas(self, previous_pos: DevicePosition):  # now - last
        self.time_delta = self.mean_time_rel - previous_pos.mean_time_rel
        self.pos_delta = {k: self.pos_abs.get(k, 0) - previous_pos.pos_abs.get(k, 0) for k in
                          set(self.pos_abs) & set(previous_pos.pos_abs)}
        # calculate relative rotation from last point
        # q_curr = q_delta * q_last
        # q_curr * q_last^-1 = q_delta
        self.rot_delta = (self.rot_abs * (previous_pos.rot_abs.inv()))

    def set_position_global(self, previous_pos: DevicePosition):
        self.pos_abs = {k: self.pos_delta.get(k, 0) + previous_pos.pos_abs.get(k, 0) for k in
                        set(self.pos_delta) & set(previous_pos.pos_abs)}
        self.rot_abs = self.rot_delta * previous_pos.rot_abs


class MeasurementInstance:  # Single line of scanner measurements along information about device position
    def __init__(self, input_position):
        """ Input structure for device curve
        input_position = {'mean_time_rel': 0.0,
                                'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                'dev_rot_vect': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                'scanner_points': [{'X': 0.0, 'Y': 0.0, 'Z': 0.0}]}
                                scanner points must be in device frame as it won't change"""
        self.position = DevicePosition(mean_time_rel=input_position['mean_time_rel'],
                                       dev_pos=input_position['dev_pos'],
                                       dev_rot_vect=input_position['dev_rot_vect'])
        self.scanner_points = []
        for point in input_position['scanner_points']:
            self.scanner_points.append(ScannerPoint(point))

    def calc_rel_points_vect(self):
        for point in self.scanner_points:
            point.calculate_relative_vect(self.position.rot_abs)

    def calc_glob_points_vect(self):
        for point in self.scanner_points:
            point.calculate_global_vect(self.position.pos_abs)

    def calc_rel_glob_points_vect(self):
        for point in self.scanner_points:
            point.calculate_relative_vect(self.position.rot_abs)
            point.calculate_global_vect(self.position.pos_abs)


class DeviceCurve:  # Single instance of scanner use from start to finish
    def __init__(self, input_structure):
        """ Input structure for device curve
        input_structure = [{'mean_time_rel': 0.0,
                            'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                            'dev_rot_vect': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                            'scanner_points': [{'X': 0.0, 'Y': 0.0, 'Z': 0.0}]}]
                            scanner points must be in device frame as it won't change
                            """

        # Set things up from raw data
        self.measurements = []  # list of MeasurementInstances
        # Create list of device positions (absolute device coordinates)
        for pos in input_structure:
            self.measurements.append(MeasurementInstance(pos))

        # Fill position objects with relative changes, to last position
        # No need to complicate this as I should use it only once at initialization
        # Start with zeros, to initialize first MeasurementInstance properly
        # Making first element point on itself feels fishy, so I make a dummy temporary object
        pos_last = DevicePosition(mean_time_rel=0.0,
                                  dev_pos={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_rot_vect={'X': 0.0, 'Y': 0.0, 'Z': 0.0})

        for meas in self.measurements:
            meas.position.set_position_deltas(previous_pos=pos_last)
            pos_last = meas.position

        # Calculate points global variables
        self.actualize_points_rel_glob()
        # Create variable for voxel tree
        self.root_voxel: Voxel = Voxel(100, 0.1)

    def actualize_points_relative(self):
        for meas in self.measurements:
            meas.calc_rel_points_vect()

    def actualize_points_glob(self):
        for meas in self.measurements:
            meas.calc_glob_points_vect()

    def actualize_points_rel_glob(self):
        for meas in self.measurements:
            meas.calc_rel_glob_points_vect()

    def voxelize(self):
        self.root_voxel = Voxel(100, 0.05)
        self.root_voxel.fill_root_voxel(self.measurements)
        self.root_voxel.divide_voxel()

    def export_datastructure(self):
        # Make input datastructure back from out classes
        export_ds = []
        for meas in self.measurements:
            rot_abs = meas.position.rot_abs.as_rotvec()
            export_ds.append({'mean_time_rel': meas.position.mean_time_rel,
                              'dev_pos': meas.position.pos_abs,
                              'dev_rot_vect': {'X': rot_abs[0], 'Y': rot_abs[1], 'Z': rot_abs[2]},
                              'scanner_points': [x.pos_dev_vect for x in meas.scanner_points]})
        return export_ds


if __name__ == '__main__':
    import json

    with open(r"/media/adamw/DATA/Projekty/Praca_mgr/scan_3d_MGR/gui_server/DataAnalysisClasses/test.json",
              "r") as read_file:
        test_data = json.load(read_file)
    # print(test_data[0])
    device_curve = DeviceCurve(test_data)
    device_curve.voxelize()
    total_nodes = 0
    total_points = 0
    device_curve.root_voxel.count_nodes()
    device_curve.root_voxel.count_points()
    print('Before')
    print('Nodes: ', total_nodes)
    print('Points: ', total_points)
    device_curve.root_voxel.drop_unneeded_nodes()
    total_nodes = 0
    total_points = 0
    device_curve.root_voxel.count_nodes()
    device_curve.root_voxel.count_points()
    print('After')
    print('Nodes: ', total_nodes)
    print('Points: ', total_points)

    export_data = device_curve.export_datastructure()
    with open('/media/adamw/DATA/Projekty/Praca_mgr/scan_3d_MGR/gui_server/DataAnalysisClasses/test2.json',
              'w') as fout:
        json.dump(export_data, fout)
