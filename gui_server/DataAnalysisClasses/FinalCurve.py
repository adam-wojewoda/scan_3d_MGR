from __future__ import annotations

from typing import Dict

from scipy.spatial.transform import Rotation
from scipy.optimize import curve_fit, fmin
from numpy import nan, array, transpose, cross
from .DynamicsModel import DynamicsModel
from copy import deepcopy
from dataclasses import dataclass, field
from random import sample


class Voxel:  # Voxel structure
    def __init__(self, min_points, min_dimension):
        self.minimal_points_number = min_points
        self.minimal_voxel_dimension = min_dimension
        self.points: list[ScannerPoint] = []  # list of points references
        self.mean_plane = None  # mean voxel plane
        self.plane_needed = False
        self.sub_voxels: list[Voxel] = []  # list of sub_voxels
        self.range: Dict[str, float] = {'x_min': nan, 'x_max': nan,  # <x_min;x_max)
                                        'y_min': nan, 'y_max': nan,  # <y_min;y_max)
                                        'z_min': nan, 'z_max': nan}  # <z_min;z_max)

    @staticmethod
    def plane_dist_func(in_point, a, b, c, d):
        # Ideal plane with such definition would always return zero, however our function will return something else
        # This something else is the distance between input point and plane
        denominator = 1 / (a ** 2 + b ** 2 + c ** 2) ** 0.5
        dist = (in_point[0, :] * a
                + in_point[1, :] * b
                + in_point[2, :] * c
                + d) * denominator  # <- I keep "d" constrained,
        # the parameters would be scalable otherwise (no change in output)
        return dist

    @staticmethod
    def mean_err_abs_wrapper(params, in_points):
        a, b, c, d = params
        # B = params[1]
        # C = params[2]
        denominator = 1 / (a ** 2 + b ** 2 + c ** 2) ** 0.5
        dist = sum([abs((in_point[0] * a
                         + in_point[1] * b
                         + in_point[2] * c
                         + d) * denominator) for in_point in
                    in_points])
        # <- I keep D constrained, the parameters would be scalable otherwise (no change in output)
        return dist

    @staticmethod
    def zero_force_wrapper(d_in, in_points, params):
        a, b, c = params
        d = abs(sum([((in_point[0] * a
                       + in_point[1] * b
                       + in_point[2] * c
                       + d_in)) for in_point in in_points]))
        # <- I keep D constrained, the parameters would be scalable otherwise (no change in output)
        return d

    @staticmethod
    def total_force_wrapper(in_points, params):
        a, b, c, d = params
        d_out = sum([((in_point[0] * a
                       + in_point[1] * b
                       + in_point[2] * c
                       + d)) for in_point in in_points])
        # <- I keep D constrained, the parameters would be scalable otherwise (no change in output)
        return d_out

    def plane_in_voxel(self, plane_definition: dict[str: float]):  # Checking only opposite vertices
        edges = [
            # X+ Y+ Z+ / X- Y- Z-
            {'p1': {'X': 'x_max', 'Y': 'y_max', 'Z': 'z_max'},  # xXyz
             'p2': {'X': 'x_max', 'Y': 'y_min', 'Z': 'z_min'}},
            # X+ Y+ Z- / X- Y- Z+
            {'p1': {'X': 'x_max', 'Y': 'y_max', 'Z': 'z_min'},  # xXyZ
             'p2': {'X': 'x_min', 'Y': 'y_min', 'Z': 'z_max'}},
            # X+ Y- Z+ / X- Y+ Z-
            {'p1': {'X': 'x_max', 'Y': 'y_min', 'Z': 'z_max'},  # xXYz
             'p2': {'X': 'x_min', 'Y': 'y_max', 'Z': 'z_min'}},
            # X- Y+ Z+ / X+ Y- Z-
            {'p1': {'X': 'x_min', 'Y': 'y_max', 'Z': 'z_max'},  # xXYZ
             'p2': {'X': 'x_max', 'Y': 'y_min', 'Z': 'z_min'}}
        ]
        for edge in edges:
            d1 = self.range[edge['p1']['X']] * plane_definition['A'] + \
                 self.range[edge['p1']['Y']] * plane_definition['B'] + \
                 self.range[edge['p1']['Z']] * plane_definition['C'] + \
                 plane_definition['D']
            d2 = self.range[edge['p2']['X']] * plane_definition['A'] + \
                 self.range[edge['p2']['Y']] * plane_definition['B'] + \
                 self.range[edge['p2']['Z']] * plane_definition['C'] + \
                 plane_definition['D']

            if (d1 * d2) <= 0.0:
                return True
        return False

    def fit_plane(self):
        # Z = a1 * X + a2 * Y + c <- This format will have problems with vertical surfaces
        # A(x−x0)+B(y−y0)+C(z−z0)=0 <- This would require me to know single point of surface in advance
        # Ax+By+Cz+D=0 <- I should start with this form
        x_dat = []
        if len(self.points) > 4:
            # print(self.points)
            for point in self.points:
                x_dat.append([point.pos_glob_vect['X'], point.pos_glob_vect['Y'], point.pos_glob_vect['Z']])
            # print(x_dat)
            # print(len(self.points))
            try:
                popt, _ = curve_fit(self.plane_dist_func, transpose(array(x_dat)), [0.0] * len(self.points))
                # print(popt)
                # Find plane orientation
                # popt = fmin(self.mean_err_abs_wrapper,
                # array([1,1,1]), args=(x_dat,), xtol=1e-4, ftol=1e-4, maxiter=75,
                #             maxfun=250, full_output=0, disp=0, retall=0, callback=None, initial_simplex=None)
                vect_norm_div = 1 / (popt[0] ** 2 + popt[1] ** 2 + popt[2] ** 2) ** 0.5

                # move plane to get zero mean distance
                a, b, c = popt[0] * vect_norm_div, popt[1] * vect_norm_div, popt[2] * vect_norm_div
                d = array([vect_norm_div])
                # print(self.total_force_wrapper(x_dat, (a, b, c, d[0])))
                d = fmin(self.zero_force_wrapper, d, args=(x_dat, (a, b, c)), xtol=5e-6, ftol=5e-4,
                         maxiter=100,
                         maxfun=200, full_output=0, disp=0, retall=0, callback=None, initial_simplex=None)
                # print(self.total_force_wrapper(x_dat,(a,b,c,d[0])))
                mean_plane_tmp = {'A': a,
                                  'B': b,
                                  'C': c,
                                  'D': d[0]}

                if nan not in mean_plane_tmp.values() and self.plane_in_voxel(plane_definition=mean_plane_tmp):
                    self.mean_plane = mean_plane_tmp

                else:
                    self.mean_plane = None

            except RuntimeError as e:
                print('Failed to create plane: ', e)
                self.mean_plane = None  # popt[3]}

    def calc_points_opt_vect(self, attraction_coefficient: float):
        if self.mean_plane is None:
            for v in self.sub_voxels:
                v.calc_points_opt_vect(attraction_coefficient=attraction_coefficient)
        else:
            if attraction_coefficient is not None and attraction_coefficient != 0.0 and len(self.points) > 0:
                mass_center = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
                for p in self.points:
                    mass_center['X'] += p.pos_glob_vect['X']
                    mass_center['Y'] += p.pos_glob_vect['Y']
                    mass_center['Z'] += p.pos_glob_vect['Z']
                mass_center['X'] = mass_center['X'] / len(self.points)
                mass_center['Y'] = mass_center['Y'] / len(self.points)
                mass_center['Z'] = mass_center['Z'] / len(self.points)
            else:
                mass_center = None

            for p in self.points:
                p.calculate_plane_point_vector(self.mean_plane, attraction_coefficient, mass_center)

    def check_plane_needed(self):
        self.plane_needed = False
        if self.points:
            for point in self.points:
                if point.conf.optimize:
                    self.plane_needed = True
                    break

    def fit_all_planes(self, all_fit=False):
        if not self.points:
            self.plane_needed = False
            for v in self.sub_voxels:
                v.fit_all_planes(all_fit)
        else:
            for point in self.points:
                point.conf.opt_vect_actual = False
            self.check_plane_needed()
            if len(self.points) >= 5 and (self.plane_needed or all_fit):
                self.fit_plane()
            else:
                self.plane_needed = False
                self.mean_plane = None

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

    def fill_root_voxel(self, measurements_in: list[MeasurementInstance], sample_in: float = 1.0, points_use=None):
        self.points = []
        if points_use is not None:
            # get total points number
            tmp = 0
            for meas in measurements_in:
                for point in meas.scanner_points:  # <--- add points sampling
                    if point.conf.pos_abs_actual:
                        tmp += 1
            sample_in = min(1.0, points_use / tmp)

        for meas in measurements_in:
            # if meas.config['scanner_points_abs_glob_actual']:
            for point in sample(meas.scanner_points,
                                int(len(meas.scanner_points) * sample_in)):  # <--- add points sampling
                if point.conf.pos_abs_actual:
                    self.points.append(point)

        len_tot = 0
        for meas in measurements_in:
            len_tot += len(meas.scanner_points)

        # print('Points to Voxelize: ', len(self.points))
        # print('Total points: ', len_tot)
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

        if d_max > self.minimal_voxel_dimension or len(self.points) > self.minimal_points_number:
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
        # In other words EAT USELESS CHILDREN and keep the gran-kids
        # end = False
        while True:
            if self.points:
                # end = True
                return
            else:
                if len(self.sub_voxels) == 1:
                    if self.sub_voxels[0].points:
                        # end = True
                        return
                    else:
                        # steal grandchildren
                        self.sub_voxels: list[Voxel] = self.sub_voxels[0].sub_voxels

                else:
                    for v in self.sub_voxels:
                        v.drop_unneeded_nodes()
                        # end = True
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
        # global total_nodes
        # total_nodes = total_nodes + 1
        nodes_here_and_below = 1
        if not self.points:
            for v in self.sub_voxels:
                nodes_here_and_below = nodes_here_and_below + v.count_nodes()
        return nodes_here_and_below

    def count_points(self):
        # global total_points
        points_here_and_below = 0
        if self.points:
            points_here_and_below = len(self.points)
        else:
            for v in self.sub_voxels:
                points_here_and_below = points_here_and_below + v.count_points()
        return points_here_and_below

    def get_subdivisions(self, level_in):
        list_of_ranges = [{'level': level_in + 1, 'ranges': self.range.copy()}]
        if not self.points:
            for v in self.sub_voxels:
                list_of_ranges.extend(v.get_subdivisions(level_in + 1))
        return list_of_ranges

    def plane_between_points(self, point_1, point_2):
        # Ideal plane with such definition would always return zero, however our function will return something else
        # This something else is the distance between input point and plane times 6^0.5
        # 1 = (A ** 2 + B ** 2 + C ** 2) ** 0.5 <- input plane should have this normalized
        d1 = point_1['X'] * self.mean_plane['A'] + \
             point_1['Y'] * self.mean_plane['B'] + \
             point_1['Z'] * self.mean_plane['C'] + \
             self.mean_plane['D']
        d2 = point_2['X'] * self.mean_plane['A'] + \
             point_2['Y'] * self.mean_plane['B'] + \
             point_2['Z'] * self.mean_plane['C'] + \
             self.mean_plane['D']

        return (d1 * d2) <= 0.0

    def voxel_plane_points(self):
        edges = [
            {'var': 'X', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_min'], 'Z': self.range['z_min']},  # xXyz
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_min'], 'Z': self.range['z_min']}},
            {'var': 'X', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_min'], 'Z': self.range['z_max']},  # xXyZ
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_min'], 'Z': self.range['z_max']}},
            {'var': 'X', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_max'], 'Z': self.range['z_min']},  # xXYz
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_max'], 'Z': self.range['z_min']}},
            {'var': 'X', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_max'], 'Z': self.range['z_max']},  # xXYZ
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_max'], 'Z': self.range['z_max']}},

            {'var': 'Y', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_min'], 'Z': self.range['z_min']},  # xyYz
             'p2': {'X': self.range['x_min'], 'Y': self.range['y_max'], 'Z': self.range['z_min']}},
            {'var': 'Y', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_min'], 'Z': self.range['z_max']},  # xyYZ
             'p2': {'X': self.range['x_min'], 'Y': self.range['y_max'], 'Z': self.range['z_max']}},
            {'var': 'Y', 'p1': {'X': self.range['x_max'], 'Y': self.range['y_min'], 'Z': self.range['z_min']},  # XyYz
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_max'], 'Z': self.range['z_min']}},
            {'var': 'Y', 'p1': {'X': self.range['x_max'], 'Y': self.range['y_min'], 'Z': self.range['z_max']},  # XyYZ
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_max'], 'Z': self.range['z_max']}},

            {'var': 'Z', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_min'], 'Z': self.range['z_min']},  # xyzZ
             'p2': {'X': self.range['x_min'], 'Y': self.range['y_min'], 'Z': self.range['z_max']}},
            {'var': 'Z', 'p1': {'X': self.range['x_min'], 'Y': self.range['y_max'], 'Z': self.range['z_min']},  # xYzZ
             'p2': {'X': self.range['x_min'], 'Y': self.range['y_max'], 'Z': self.range['z_max']}},
            {'var': 'Z', 'p1': {'X': self.range['x_max'], 'Y': self.range['y_min'], 'Z': self.range['z_min']},  # XyzZ
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_min'], 'Z': self.range['z_max']}},
            {'var': 'Z', 'p1': {'X': self.range['x_max'], 'Y': self.range['y_max'], 'Z': self.range['z_min']},  # XYzZ
             'p2': {'X': self.range['x_max'], 'Y': self.range['y_max'], 'Z': self.range['z_max']}},
        ]
        if self.mean_plane is None:
            return []
        else:
            plane_points = []
            for edge in edges:
                if self.plane_between_points(point_1=edge['p1'], point_2=edge['p2']):
                    if edge['var'] == 'X':
                        # Ax+By+Cz+D=0 -> x=-(By+Cz+D)/A
                        x_temp = -(self.mean_plane['B'] * edge['p1']['Y'] + self.mean_plane['C'] * edge['p1']['Z'] +
                                   self.mean_plane['D']) / self.mean_plane['A']

                        plane_points.append({'X': x_temp, 'Y': edge['p1']['Y'], 'Z': edge['p1']['Z']})
                    elif edge['var'] == 'Y':
                        # Ax+By+Cz+D=0 -> y=-(Ax+Cz+D)/B
                        y_temp = -(self.mean_plane['A'] * edge['p1']['X'] + self.mean_plane['C'] * edge['p1']['Z'] +
                                   self.mean_plane['D']) / self.mean_plane['B']
                        plane_points.append({'X': edge['p1']['X'], 'Y': y_temp, 'Z': edge['p1']['Z']})
                    elif edge['var'] == 'Z':
                        # Ax+By+Cz+D=0 -> z=-(Ax+By+D)/C
                        z_temp = -(self.mean_plane['A'] * edge['p1']['X'] + self.mean_plane['B'] * edge['p1']['Y'] +
                                   self.mean_plane['D']) / (self.mean_plane['C'])
                        plane_points.append({'X': edge['p1']['X'], 'Y': edge['p1']['Y'], 'Z': z_temp})
                    else:
                        raise KeyError('var should be X, Y or Z (only checking orthogonal lines)')
            return [{'vox': plane_points}] if plane_points else []

    def get_planes(self):
        list_of_planes = []
        if self.mean_plane is None and self.sub_voxels:  # We have sub-voxels
            for v in self.sub_voxels:
                list_of_planes.extend(v.get_planes())
        else:  # We have point
            if self.mean_plane is not None:
                list_of_planes.extend(self.voxel_plane_points())
        return list_of_planes

    def get_quiver(self):
        list_of_points = []
        if not self.points:
            for v in self.sub_voxels:
                list_of_points.extend(v.get_quiver())
        else:
            for p in self.points:
                list_of_points.append([p.pos_glob_vect['X'],
                                       p.pos_glob_vect['Y'],
                                       p.pos_glob_vect['Z'],
                                       p.opt_vect['X'],
                                       p.opt_vect['Y'],
                                       p.opt_vect['Z']])
        return list_of_points


@dataclass
class PointState:
    optimize = False
    pos_abs_actual = False
    pos_rel_actual = False
    opt_vect_actual = False


class ScannerPoint:  # Single scanner point
    def __init__(self, point_in: dict):
        # self.voxel_reference:Voxel = None
        self.pos_glob_vect: dict = {'X': nan, 'Y': nan, 'Z': nan}  # global point position (earth frame)
        self.pos_rel_vect: dict = {'X': nan, 'Y': nan, 'Z': nan}  # relative point position (earth frame)
        # self.pos_dev_vect = {'X': None, 'Y': None, 'Z': None}  # device frame point position (device frame)
        self.pos_dev_vect: dict = point_in.copy()
        self.opt_vect: dict = {'X': nan, 'Y': nan, 'Z': nan}  # optimization force vector
        self.conf = PointState()

    def calculate_relative_vect(self, rot_in: Rotation):
        tmp = rot_in.apply([self.pos_dev_vect['X'], self.pos_dev_vect['Y'], self.pos_dev_vect['Z']], inverse=False)
        self.pos_rel_vect = {'X': tmp[0], 'Y': tmp[1], 'Z': tmp[2]}
        self.conf.pos_rel_actual = True
        self.conf.pos_abs_actual = False
        self.conf.opt_vect_actual = False

    def device_moved(self):
        self.conf.pos_rel_actual = False
        self.conf.pos_abs_actual = False
        self.conf.opt_vect_actual = False

    def calculate_global_vect(self, dev_global_pos):
        self.pos_glob_vect = {k: self.pos_rel_vect.get(k, 0) + dev_global_pos.get(k, 0) for k in
                              set(self.pos_rel_vect) & set(dev_global_pos)}
        self.conf.pos_abs_actual = True
        self.conf.opt_vect_actual = False  # Redundant operation

    def calculate_plane_point_vector(self, plane: Dict, attraction_coefficient: float, mass_center: dict):
        # Ideal plane with such definition would always return zero, however our function will return something else
        # This something else is the distance between input point and plane times 6^0.5
        # 1 = (A ** 2 + B ** 2 + C ** 2) ** 0.5 <- input plane should have this normalized
        if plane is not None and None not in plane.values():
            d = self.pos_glob_vect['X'] * plane['A'] \
                + self.pos_glob_vect['Y'] * plane['B'] \
                + self.pos_glob_vect['Z'] * plane['C'] \
                + plane['D']

            self.opt_vect['X'] = - plane['A'] * d
            self.opt_vect['Y'] = - plane['B'] * d
            self.opt_vect['Z'] = - plane['C'] * d

            if mass_center is not None and (attraction_coefficient is not None and attraction_coefficient != 0.0):
                # Attraction force is directed to mass center of equally weighted points
                # Due to the way plane position is calculated it should lay on it therefore
                # resultant force perpendicular to plane will be equal to zero
                # Forces are proportional to distance from mass center
                self.opt_vect['X'] += (mass_center['X'] - self.pos_glob_vect['X']) * attraction_coefficient
                self.opt_vect['Y'] += (mass_center['Y'] - self.pos_glob_vect['Y']) * attraction_coefficient
                self.opt_vect['Z'] += (mass_center['Z'] - self.pos_glob_vect['Z']) * attraction_coefficient

        else:
            self.opt_vect['X'] = 0.0
            self.opt_vect['Y'] = 0.0
            self.opt_vect['Z'] = 0.0
        self.conf.opt_vect_actual = True

    # def calc_size_opt_vect(self, cube_ranges: Dict[str:float]):
    #     # Move positions to fit in dimension cube, rotations not allowed
    #     # Algorithm must be simple, so I've resigned from using sphere
    #     # first point (device position) is immovable
    #     # {'x_min': nan, 'x_max': nan,  # <x_min;x_max)
    #     #  'y_min': nan, 'y_max': nan,  # <y_min;y_max)
    #     #  'z_min': nan, 'z_max': nan}  # <z_min;z_max)
    #     if self.pos_glob_vect['X'] > cube_ranges['x_max']:
    #         self.opt_vect['X'] = cube_ranges['x_max'] - self.pos_glob_vect['X']
    #     else:
    #         if self.pos_glob_vect['X'] < cube_ranges['x_min']:
    #             self.opt_vect['X'] = cube_ranges['x_in'] - self.pos_glob_vect['X']
    #         else:
    #             self.opt_vect['X'] = 0.0
    #
    #     if self.pos_glob_vect['Y'] > cube_ranges['y_max']:
    #         self.opt_vect['Y'] = cube_ranges['y_max'] - self.pos_glob_vect['Y']
    #     else:
    #         if self.pos_glob_vect['Y'] < cube_ranges['y_min']:
    #             self.opt_vect['Y'] = cube_ranges['y_in'] - self.pos_glob_vect['Y']
    #         else:
    #             self.opt_vect['Y'] = 0.0
    #
    #     if self.pos_glob_vect['Z'] > cube_ranges['z_max']:
    #         self.opt_vect['Z'] = cube_ranges['z_max'] - self.pos_glob_vect['Z']
    #     else:
    #         if self.pos_glob_vect['Z'] < cube_ranges['z_min']:
    #             self.opt_vect['Z'] = cube_ranges['z_in'] - self.pos_glob_vect['Z']
    #         else:
    #             self.opt_vect['Z'] = 0.0
    #     pass


@dataclass
class OptVectorDev:  # Optimization force vector structure
    pos_loc_vect: dict[str, float] = field(
        default_factory=lambda: ({'X': nan, 'Y': nan, 'Z': nan}))  # local needed position change 'force'
    ang_loc_vect: dict[str, float] = field(
        default_factory=lambda: ({'X': nan, 'Y': nan, 'Z': nan}))  # local needed rotation change 'force' (rot_vect)
    pos_glob_vect: dict[str, float] = field(
        default_factory=lambda: ({'X': nan, 'Y': nan, 'Z': nan}))  # global needed position change 'force'
    ang_glob_vect: dict[str, float] = field(
        default_factory=lambda: ({'X': nan, 'Y': nan, 'Z': nan}))  # global needed rotation change 'force' (rot_vect)


@dataclass
class DevPositionState:
    pos_delta_actual = False
    pos_abs_actual = False
    opt_loc_actual = False
    opt_force_actual = False


class DevicePosition:  # Everything regarding device position in space + optimization vector instance
    def __init__(self, mean_time_rel, dev_speed, dev_pos, dev_rot_vect, dev_euler_ang):
        self.mean_time_rel = mean_time_rel
        self.conf = DevPositionState()
        self.time_delta = nan  # time from last position
        self.pos_delta = {'X': nan, 'Y': nan, 'Z': nan}  # translation from last point in [m]
        self.rot_delta = None
        if dev_pos is not None:  # absolute linear position [m]
            self.pos_abs = dev_pos.copy()
        else:
            self.pos_abs = None

        if dev_speed is not None:  # absolute linear speed [m/s]
            self.speed_abs = dev_speed.copy()
        else:
            self.speed_abs = None

        if dev_rot_vect is not None:  # abs. scanner rot. vect [rad] (rot. of coordinate system)
            self.rot_abs = Rotation.from_rotvec([dev_rot_vect['X'], dev_rot_vect['Y'], dev_rot_vect['Z']])
        else:
            self.rot_abs = None

        if dev_euler_ang is not None:  # absolute scanner euler angles[deg]
            self.euler_ang = dev_euler_ang.copy()
        else:
            self.euler_ang = None

        # self.opt_vect = deepcopy(OptVectorDev())
        self.opt_vect = OptVectorDev()

    def set_position_deltas(self, previous_pos: DevicePosition):  #
        self.time_delta = self.mean_time_rel - previous_pos.mean_time_rel
        self.pos_delta = {k: self.pos_abs.get(k, 0) - previous_pos.pos_abs.get(k, 0) for k in
                          set(self.pos_abs) & set(previous_pos.pos_abs)}
        self.speed_abs = {k: self.pos_delta.get(k, 0) / self.time_delta for k in self.pos_delta}
        # calculate relative rotation from last point
        # q_curr = q_delta * q_last
        # q_curr * q_last^-1 = q_delta
        self.rot_delta = (self.rot_abs * (previous_pos.rot_abs.inv()))
        self.conf.pos_delta_actual = True

    def set_position_global(self, previous_pos: DevicePosition):
        self.pos_abs = {k: self.pos_delta.get(k, 0) + previous_pos.pos_abs.get(k, 0) for k in
                        set(self.pos_delta) & set(previous_pos.pos_abs)}
        self.speed_abs = {k: self.pos_delta.get(k, 0) / self.time_delta for k in self.pos_delta}
        self.rot_abs = self.rot_delta * previous_pos.rot_abs
        self.conf.pos_abs_actual = True
        self.conf.opt_loc_actual = False
        self.conf.opt_force_actual = False

    def get_loc_opt_vect(self, point_list: list[ScannerPoint], rot=False):
        ax = ['X', 'Y', 'Z']
        self.opt_vect.pos_loc_vect = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.opt_vect.ang_loc_vect = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        points_no_opt = 0
        for point in point_list:
            if nan not in point.opt_vect.values() and point.conf.optimize and point.conf.opt_vect_actual:
                if not point.conf.opt_vect_actual:
                    points_no_opt += 1

                for axis in ax:
                    self.opt_vect.pos_loc_vect[axis] += point.opt_vect[axis]

                if rot:
                    r = {'X': point.pos_glob_vect['X'] - self.pos_abs['X'],
                         'Y': point.pos_glob_vect['Y'] - self.pos_abs['Y'],
                         'Z': point.pos_glob_vect['Z'] - self.pos_abs['Z']}
                    tmp = cross([r[x] for x in ax], [point.opt_vect[x] for x in ax])

                    for axis, arg in zip(ax, tmp):
                        self.opt_vect.ang_loc_vect[axis] += arg
        if points_no_opt > 0:
            print('Found ', points_no_opt, ' points without optimization vector!')
        self.conf.opt_loc_actual = True
        self.conf.opt_force_actual = False

    def get_state(self):
        rot_vect = self.rot_abs.as_rotvec()

        return {'time_rel': self.mean_time_rel,
                'dev_pos': self.pos_abs,
                'dev_speed': self.speed_abs,
                'dev_rot_vect': {'X': rot_vect[0], 'Y': rot_vect[1], 'Z': rot_vect[2]},
                'dev_euler_ang': self.euler_ang}

    def set_state(self, state_in):
        self.mean_time_rel = state_in['time_rel']
        self.pos_abs = deepcopy(state_in['dev_pos'])
        self.speed_abs = deepcopy(state_in['dev_speed'])
        self.rot_abs = Rotation.from_rotvec([state_in['dev_rot_vect']['X'],
                                             state_in['dev_rot_vect']['Y'],
                                             state_in['dev_rot_vect']['Z']])
        self.euler_ang = deepcopy(state_in['dev_euler_ang'])
        self.conf.pos_abs_actual = True
        self.conf.pos_delta_actual = False
        self.conf.opt_loc_actual = False
        self.conf.opt_force_actual = False


@dataclass
class MeasurementState:
    optimize_pos = False
    dev_pos_from_opt = False
    available = False


class MeasurementInstance:  # Single line of scanner measurements along information about device position
    def __init__(self, input_position, dynamic_model_use):
        """ Input structure for device curve
        input_position = {'mean_time_rel': 0.0,
                            'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                            'dev_speed': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                            'dev_rot_vect': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                            'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
                            'imu_input': [{'rel_time': 0.0,
                                'X_acc': 0.0, 'Y_acc': 0.0, 'Z_acc': 0.0,
                                'X_gyr': 0.0, 'Y_gyr': 0.0, 'Z_gyr': 0.0}]
                            'scanner_points': [{'X': 0.0, 'Y': 0.0, 'Z': 0.0}]}
                                scanner points must be in device frame as it won't change"""

        if (not dynamic_model_use) and (
                (input_position['dev_pos'] is None) or (
                input_position['dev_rot_vect'] is None) or (
                        input_position['dev_speed'] is None) or (
                        input_position['dev_euler_ang'] is None)):
            raise ValueError('Device positions and speed needed when not using dynamic model!')
        if input_position['time_rel'] is None:
            raise ValueError('Measurement time not specified!')
        if dynamic_model_use and (input_position['imu_input'] is None):
            raise ValueError('IMU data needed for use with dynamic model!')
        if (input_position['scanner_points'] is None) or len(input_position['scanner_points']) == 0:
            raise ValueError('No scanner points for measurement instance!')
        self.conf = MeasurementState()
        # self.config = {'rel_dev_pos_actual': False,  # Dev pos relative to previous measurement
        #                # 'rel_dev_pos_available': False,
        #                'abs_dev_pos_actual': False,  # Absolute dev pos
        #                # 'abs_dev_pos_available': False,
        #                'scanner_points_rel_glob_actual': False,  # Scanner points relative to dev pos in earth frame
        #                'scanner_points_abs_glob_actual': False,  # Scanner points in absolute coordinates
        #                # 'opt_vect_points_actual': False,
        #                # 'opt_vect_dev_loc_actual': False,
        #                # 'opt_vect_dev_glob_actual': False,
        #                'optimize_pos': False,  # Do we change the position through optimization
        #                'dev_pos_from_optimization': False}  # Does the dev pos come from optimization or Kalman

        self.position = DevicePosition(mean_time_rel=input_position['time_rel'],
                                       dev_pos=input_position['dev_pos'],
                                       dev_speed=input_position['dev_speed'],
                                       dev_rot_vect=input_position['dev_rot_vect'],
                                       dev_euler_ang=input_position['dev_euler_ang'])

        if dynamic_model_use:
            self.imu_data = input_position['imu_input'].copy()  # IMU measurements preceding scanner measurement
            # for point in input_position['imu_input']:
            #     self.imu_data.append(point)
        else:
            self.imu_data = None

        self.scanner_points = []
        for point in input_position['scanner_points']:
            self.scanner_points.append(ScannerPoint(point))

    # def apply_maximal_position_change(self, specific_speed=1.0):
    #     # That is the easiest part (needs only 1 iteration), rotations not allowed
    #     max_dist = specific_speed * self.position.time_delta
    #     dist_squared = self.position.pos_delta['X'] ** 2 + self.position.pos_delta['Y'] ** 2 + self.position.pos_delta[
    #         'Z'] ** 2
    #     if dist_squared > max_dist ** 2:
    #         # shorten position delta vector
    #         denominator = max_dist / dist_squared ** 0.5
    #         self.position.pos_delta['X'] = self.position.pos_delta['X'] * denominator
    #         self.position.pos_delta['Y'] = self.position.pos_delta['Y'] * denominator
    #         self.position.pos_delta['Z'] = self.position.pos_delta['Z'] * denominator

    def calc_loc_opt_vect(self, rot=False):
        # Use points and flat planes in voxels, rotations applicable
        self.position.get_loc_opt_vect(self.scanner_points, rot=rot)

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

    def set_position_state(self, pos, optimize):
        self.position.set_state(pos)
        for point in self.scanner_points:
            point.conf.pos_abs_actual = False
            point.conf.pos_rel_actual = False
            point.conf.opt_vect_actual = False
            point.conf.optimize = optimize


@dataclass
class PosIndexState:
    last_filled_pos_index = None  # Index of last measurement with filled device position
    last_opt_pos_index = None  # Index of last optimised position
    kalman_index_now = None  # Current index of Kalman filter
    kalman_index_init = None  # Index of measurement with initial covariance


class DeviceCurve:  # Single instance of scanner use from start to finish
    def __init__(self, input_structure, dynamic_model_use=True, init_ang=None, g_in=9.8159, dt_in=0.001):
        """ Input structure for device curve
        input_structure = [{'mean_time_rel': 0.0,
                           'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                           'dev_speed': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                           'dev_rot_vect': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                           'dev_euler_ang': {'psi': 0.0, 'theta': 0.0, 'phi': 0.0},
                           'imu_input':
                              [{'mean_time_rel': 0.0,
                                'X_acc': 0.0, 'Y_acc': 0.0, 'Z_acc': 0.0,
                                'X_gyr': 0.0, 'Y_gyr': 0.0, 'Z_gyr': 0.0}],
                            'scanner_points': [{'X': 0.0, 'Y': 0.0, 'Z': 0.0}]}]
                            scanner points must be in device frame as it won't change
                            """
        self.using_dynamics = dynamic_model_use
        # self.opt_vect_point_actual = False
        # self.opt_vect_dev_actual = False
        # self.opt_forces_actual = False
        # self.opt_surfaces_actual = False
        self.total_nodes = 0  # Total voxel nodes
        self.total_points = 0  # Total scanner points
        self.indexes = PosIndexState()
        # self.last_filled_pos_index = None  # Index of last measurement with filled device position
        # self.last_opt_pos_index = None  # Index of last optimised position
        # self.kalman_index_now = None  # Current index of Kalman filter
        # self.kalman_index_init = None  # Index of measurement with initial covariance
        # Set things up from raw data
        self.measurements = []  # list of MeasurementInstances
        if init_ang is None:  # Initial euler angles
            self.init_ang = {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
        else:
            self.init_ang = init_ang
        # Create list of device positions (absolute device coordinates)
        for pos in input_structure:
            self.measurements.append(MeasurementInstance(pos, dynamic_model_use))

        if dynamic_model_use:
            # Create instance of dynamics model with which we'll construct the curve
            self.dynamics_model: DynamicsModel = DynamicsModel(g_in=g_in, dt_in=dt_in)
            if init_ang is None:
                self.init_ang = {'psi': 0.0, 'theta': 0.0, 'phi': 0.0}
            else:
                self.init_ang = init_ang
        else:
            # Fill position objects with relative changes, to last position
            # No need to complicate this as I should use it only once at initialization
            # Start with zeros, to initialize first MeasurementInstance properly
            # Making first element point on itself feels fishy, so I make a dummy temporary object
            self.dynamics_model = None

            pos_last = DevicePosition(mean_time_rel=0.0,
                                      dev_pos={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                      dev_speed={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                      dev_rot_vect={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                      dev_euler_ang={'psi': 0.0, 'theta': 0.0, 'phi': 0.0})

            for meas in self.measurements:
                meas.position.set_position_deltas(previous_pos=pos_last)
                pos_last = meas.position

            # Calculate points global variables
            # if not dynamic_model_use:  # with dynamic model there's nothing to calculate
            self.actualize_points_rel_glob()
        # Create variable for voxel tree
        self.root_voxel: Voxel = Voxel(100, 0.5)

    def calc_dynamic_dev_pos(self, rel: bool = True, num: int = 1) -> int:
        """

        @return: Number of new points added
        @param rel: - True - we add x new points, False - we make sure x first points are available
        @param num: - number of points to add or to make sure we have
        """
        # self.opt_vect_point_actual = False
        # self.opt_vect_dev_actual = False
        # self.opt_forces_actual = False
        # Check or initialize Kalman initial state
        if self.indexes.last_filled_pos_index is None:
            # Initialize with no stable points
            self.indexes.last_filled_pos_index = -1
            self.indexes.last_opt_pos_index = -1
            self.indexes.kalman_index_now = -1
            self.indexes.kalman_index_init = -1
            if self.dynamics_model is not None:
                self.dynamics_model.write_state(state_in={
                    'time_rel': 0.0,
                    'dev_pos': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                    'dev_speed': {'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                    'dev_rot_vect': None,
                    'dev_euler_ang': self.init_ang}, zero_point=True)
            else:
                raise RuntimeError('No dynamics model defined!')
        else:
            # initialize with stable point
            # if kalman is up-to-date, there's no need for modification
            if self.indexes.last_opt_pos_index > self.indexes.kalman_index_init:
                # Initialize in new point
                if self.dynamics_model is not None:
                    # print('Reinitialization')
                    # print(self.dynamics_model.integrator.initial_state)
                    self.dynamics_model.write_state(
                        state_in=self.measurements[self.indexes.last_opt_pos_index].position.get_state(),
                        zero_point=False)
                    # print(self.dynamics_model.integrator.initial_state)
                    self.indexes.kalman_index_now = self.indexes.last_opt_pos_index
                    self.indexes.kalman_index_init = self.indexes.last_opt_pos_index
            else:
                # No new init point
                if not ((self.indexes.last_filled_pos_index == self.indexes.kalman_index_now) or (
                        self.indexes.kalman_index_now == self.indexes.kalman_index_init)):
                    # reinitialization needed
                    self.dynamics_model.write_state(
                        state_in=self.measurements[self.indexes.last_opt_pos_index].position.get_state(),
                        zero_point=False)
                    self.indexes.kalman_index_now = self.indexes.last_opt_pos_index
                    self.indexes.kalman_index_init = self.indexes.last_opt_pos_index

        if rel:
            # Calculate how many points to run with dynamic model
            # Starting with Kalman index and running till en number of new points is met
            points_to_go = min(num + self.indexes.last_filled_pos_index - self.indexes.kalman_index_init,
                               len(self.measurements) - (self.indexes.kalman_index_now + 1))
            self.indexes.kalman_index_now = self.indexes.kalman_index_init
        else:
            points_to_go = min(num - 1 - self.indexes.kalman_index_init,
                               len(self.measurements) - (self.indexes.kalman_index_now + 1))

        points_to_go = max(0, points_to_go)
        for meas in self.measurements[self.indexes.kalman_index_init + 1:]:
            meas.conf.optimize_pos = False
            # self.measurements[self.kalman_index_now].config['opt_vect_actual'] = False
            meas.conf.dev_pos_from_opt = False
            meas.conf.available = False
        for i in range(points_to_go):
            # here we call dynamic model and put its data into device position
            # 1. run kalman on data from measurement
            # print('Calculation')
            # print(self.dynamics_model.integrator.initial_state)
            out = self.dynamics_model.kalman.apply_kalman(self.measurements[self.indexes.kalman_index_now + 1].imu_data)
            # print(self.dynamics_model.integrator.initial_state)
            self.indexes.kalman_index_now += 1
            # 2. integrate position
            pos = self.dynamics_model.integrator.integrate(input_list=out)
            # print('Output position')
            # print(pos)
            # 3. put new position into device position
            self.measurements[
                self.indexes.kalman_index_now].set_position_state(pos=pos, optimize=True)
            self.indexes.last_filled_pos_index = self.indexes.kalman_index_now
            # Actualise measurement configuration (Could do it cleaner but I don't care)
            # self.measurements[self.indexes.kalman_index_now].config['rel_dev_pos_actual'] = False
            # self.measurements[self.kalman_index_now].config['rel_dev_pos_available'] = False
            # self.measurements[self.kalman_index_now].config['abs_dev_pos_available'] = True
            # self.measurements[self.indexes.kalman_index_now].config['abs_dev_pos_actual'] = True
            # self.measurements[self.indexes.kalman_index_now].config['scanner_points_rel_glob_actual'] = False
            # self.measurements[self.indexes.kalman_index_now].config['scanner_points_abs_glob_actual'] = False
            # self.measurements[self.kalman_index_now].config['opt_vect_points_actual'] = False
            # self.measurements[self.kalman_index_now].config['opt_vect_dev_loc_actual'] = False
            # self.measurements[self.kalman_index_now].config['opt_vect_dev_glob_actual'] = False
            self.measurements[self.indexes.kalman_index_now].conf.optimize_pos = True
            # self.measurements[self.kalman_index_now].config['opt_vect_actual'] = False
            self.measurements[self.indexes.kalman_index_now].conf.dev_pos_from_opt = False
            self.measurements[self.indexes.kalman_index_now].conf.available = True
        return points_to_go

    # def apply_max_speed(self, max_speed=1.0):
    #     for meas in self.measurements:
    #         meas.apply_maximal_position_change(specific_speed=max_speed)

    def calc_points_opt_vect(self, attraction_coefficient):
        self.root_voxel.calc_points_opt_vect(attraction_coefficient=attraction_coefficient)
        # self.opt_vect_point_actual = True
        # self.opt_vect_dev_actual = False
        # self.opt_forces_actual = False
        # for meas in self.measurements:
        #     if meas.config['']

    # def fit_positions_in_cube(self, cube_ranges: Dict[str:float]):
    #     # Move positions to fit in dimension cube, rotations not allowed
    #     # Algorithm must be simple, so I've resigned from using sphere
    #     # first point (device position) is immovable
    #     # {'x_min': nan, 'x_max': nan,  # <x_min;x_max)
    #     #  'y_min': nan, 'y_max': nan,  # <y_min;y_max)
    #     #  'z_min': nan, 'z_max': nan}  # <z_min;z_max)
    #     # The best would probably be to scale all the positions to fit in cube
    #     pos_range = {'x_min': self.measurements[0].position.pos_abs['X'],
    #                  'x_max': self.measurements[0].position.pos_abs['X'],  # <x_min;x_max)
    #                  'y_min': self.measurements[0].position.pos_abs['Y'],
    #                  'y_max': self.measurements[0].position.pos_abs['Y'],  # <y_min;y_max)
    #                  'z_min': self.measurements[0].position.pos_abs['Z'],
    #                  'z_max': self.measurements[0].position.pos_abs['Z']}  # <z_min;z_max)
    #     for meas in self.measurements:
    #         pos_range['x_min'] = min(pos_range['x_min'], meas.position.pos_abs['X'])
    #         pos_range['x_max'] = max(pos_range['x_max'], meas.position.pos_abs['X'])
    #         pos_range['y_min'] = min(pos_range['y_min'], meas.position.pos_abs['Y'])
    #         pos_range['y_max'] = max(pos_range['y_max'], meas.position.pos_abs['Y'])
    #         pos_range['z_min'] = min(pos_range['z_min'], meas.position.pos_abs['Z'])
    #         pos_range['z_max'] = max(pos_range['z_max'], meas.position.pos_abs['Z'])
    #
    #     scalars = {
    #         'X': min(abs(cube_ranges['x_min'] / pos_range['x_min'] if pos_range['x_min'] != 0.0 else 1.0),
    #                  abs(cube_ranges['x_max'] / pos_range['x_max'] if pos_range['x_max'] != 0.0 else 1.0),
    #                  1.0),
    #         'Y': min(abs(cube_ranges['y_min'] / pos_range['y_min'] if pos_range['y_min'] != 0.0 else 1.0),
    #                  abs(cube_ranges['y_max'] / pos_range['y_max'] if pos_range['y_max'] != 0.0 else 1.0),
    #                  1.0),
    #         'Z': min(abs(cube_ranges['z_min'] / pos_range['z_min'] if pos_range['z_min'] != 0.0 else 1.0),
    #                  abs(cube_ranges['z_max'] / pos_range['z_max'] if pos_range['z_max'] != 0.0 else 1.0),
    #                  1.0)}
    #
    #     for meas in self.measurements:
    #         meas.position.pos_abs['X'] = meas.position.pos_abs['X'] * scalars['X']
    #         meas.position.pos_abs['Y'] = meas.position.pos_abs['Y'] * scalars['Y']
    #         meas.position.pos_abs['Z'] = meas.position.pos_abs['Z'] * scalars['Z']

    def update_loc_opt(self, rot=False):
        for meas in self.measurements:
            if meas.conf.available and meas.conf.optimize_pos:
                meas.calc_loc_opt_vect(rot=rot)
        # self.opt_vect_dev_actual = True
        # self.opt_forces_actual = False

    def move_rel_pos(self,
                     elast_lin=0.01,  # [m/N]
                     elast_rot=0.01,  # [rad/N]
                     rot=False,
                     mode='time',  # time or const
                     max_move=None
                     ):
        # self.opt_vect_point_actual = False
        # self.opt_vect_dev_actual = False
        # self.opt_forces_actual = False
        # first point stays rigid (anchored)
        # movement restriction can be linear (each next position is easier to move) or constant
        # (each point is as easy to move)
        # I restrict points movement either by applying elasticity (time dependent or constant)
        ax = ['X', 'Y', 'Z']

        # If python had references this would look way better
        if mode == 'time':
            # print('time mode')
            max_time = self.measurements[-1].position.mean_time_rel
            elast_lin_2 = elast_lin / max_time
            elast_rot_2 = elast_rot / max_time

        elif mode == 'const':
            # print('const mode')
            elast_lin_2 = elast_lin
            elast_rot_2 = elast_rot
        else:
            raise ValueError('Unknown mode')

        if max_move is None:
            move_restrict = 1
            max_move_sq = 0.0
        else:
            # iterate over table to find maximal movement of point
            tmp = {}
            max_move_sq = 0.0
            for axis in ax:
                tmp[axis] = 0.0
            if mode == 'time':
                for meas in self.measurements[1:]:
                    if meas.conf.available and meas.conf.optimize_pos:
                        max_tmp = 0.0
                        for axis in ax:
                            tmp[axis] += meas.position.opt_vect.pos_glob_vect[
                                             axis] * elast_lin_2 * meas.position.mean_time_rel
                            max_tmp += tmp[axis] ** 2
                        if max_move_sq < max_tmp:
                            max_move_sq = max_tmp

            elif mode == 'const':
                max_tmp = 0.0
                for meas in self.measurements[1:]:
                    if meas.conf.available and meas.conf.optimize_pos:
                        for axis in ax:
                            tmp[axis] += meas.position.opt_vect.pos_glob_vect[
                                             axis] * elast_lin_2
                            max_tmp += tmp[axis] ** 2
                        if max_move_sq < max_tmp:
                            max_move_sq = max_tmp
            else:
                raise ValueError('Unknown mode')
            if max_move_sq > 0.0:
                move_restrict = min(max_move / (max_move_sq ** 0.5), 1.0)
            else:
                move_restrict = 1.0
            # print('Max move old: ', max_move_sq ** 0.5)
            # print('Move scaling: ', move_restrict)
        if mode == 'time':
            for meas in self.measurements[1:]:
                if meas.conf.available and meas.conf.optimize_pos:
                    for axis in ax:
                        meas.position.pos_delta[axis] += meas.position.opt_vect.pos_glob_vect[axis] * \
                                                         elast_lin_2 * meas.position.mean_time_rel * move_restrict
        elif mode == 'const':
            for meas in self.measurements[1:]:
                if meas.conf.available and meas.conf.optimize_pos:
                    for axis in ax:
                        meas.position.pos_delta[axis] += meas.position.opt_vect.pos_glob_vect[
                                                             axis] * elast_lin_2 * move_restrict
        else:
            raise ValueError('Unknown mode')

        if rot:
            for meas in self.measurements[1:]:
                if meas.conf.available and meas.conf.optimize_pos:
                    meas.position.rot_delta = Rotation.from_rotvec(
                        [meas.position.opt_vect.ang_glob_vect['X'] * elast_rot_2,
                         meas.position.opt_vect.ang_glob_vect['Y'] * elast_rot_2,
                         meas.position.opt_vect.ang_glob_vect['Z'] * elast_rot_2]) * meas.position.rot_delta
        return max_move_sq ** 0.5, move_restrict * (max_move_sq ** 0.5)

    def update_glob_opt(self, rot=False, update_loc=False, max_speed=None, elasticity=None):
        # self.opt_forces_actual = True
        ax = ['X', 'Y', 'Z']
        if update_loc:
            self.update_loc_opt(rot=rot)
        # self.measurements.reverse()  # I revert the list for iteration purpose (No copy)

        # last_meas_time = self.measurements[0].position.mean_time_rel
        last_force = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        last_torque = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        last_dev_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}  # Global coordinates of previous dev pos

        for i in reversed(range(len(self.measurements))):  # Change to iteration only over measurements for optimization
            # update torque
            if self.measurements[i].conf.available and self.measurements[i].conf.optimize_pos and self.measurements[
                i].position.conf.opt_loc_actual:
                if rot:
                    # torque from previous point force
                    tmp = cross([(last_dev_pos[x] - self.measurements[i].position.pos_abs[x]) ** 2 for x in ax],
                                [last_force[x] for x in ax])
                    for axis, val in zip(ax, tmp):
                        last_torque[axis] += self.measurements[i].position.opt_vect.ang_loc_vect[axis] + val
                        self.measurements[i].position.opt_vect.ang_glob_vect[axis] = last_torque[axis]
                else:
                    for axis in ax:
                        last_torque[axis] += 0.0
                        self.measurements[i].position.opt_vect.ang_glob_vect[axis] = 0.0
                # update force

                for axis in ax:
                    last_force[axis] += self.measurements[i].position.opt_vect.pos_loc_vect[axis]
                    self.measurements[i].position.opt_vect.pos_glob_vect[axis] = last_force[axis]

                if (elasticity is None) != (max_speed is None):
                    raise ValueError('Only 1 parameter of 2 was given')

                if (max_speed is not None) and (elasticity is not None):
                    dist = sum([(last_dev_pos[x] - self.measurements[i].position.pos_abs[x]) ** 2 for x in ax]) ** 0.5
                    new_elasticity = max(0.0,
                                         dist - max_speed * self.measurements[
                                             i].position.time_delta) * elasticity / dist
                    if new_elasticity > 0.0:
                        for axis in ax:
                            self.measurements[i].position.opt_vect.pos_glob_vect[axis] += (
                                    - self.measurements[i].position.pos_delta[axis] * new_elasticity)
                # update position
                last_dev_pos = self.measurements[i].position.pos_abs.copy()
            self.measurements[i].position.conf.opt_force_actual = True
            # pass
            # self.measurements.reverse()  # I revert the list to previous state

    def actualize_points_relative(self):
        # self.opt_vect_dev_actual = False
        # self.opt_vect_point_actual = False
        # self.opt_forces_actual = False
        for meas in self.measurements:
            if meas.conf.available and meas.position.conf.pos_abs_actual:
                tmp = False
                for point in meas.scanner_points:
                    if not point.conf.pos_rel_actual:
                        tmp = True
                        break
                if tmp:
                    meas.calc_rel_points_vect()
                meas.position.conf.opt_loc_actual = False
                meas.position.conf.opt_force_actual = False

                # meas.config['scanner_points_rel_glob_actual'] = True

    def actualize_dev_pos_global(self):
        # self.opt_vect_dev_actual = False
        # self.opt_vect_point_actual = False
        # self.opt_forces_actual = False

        pos_last = DevicePosition(mean_time_rel=0.0,
                                  dev_pos={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_speed={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_rot_vect={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_euler_ang={'psi': 0.0, 'theta': 0.0, 'phi': 0.0})

        for meas in self.measurements:
            if meas.conf.available and meas.conf.optimize_pos:
                meas.position.set_position_global(previous_pos=pos_last)
                # meas.config['dev_pos_from_optimization'] = True
                meas.position.conf.pos_abs_actual = True
                meas.position.conf.opt_force_actual = False
                meas.position.conf.opt_loc_actual = False

                meas.conf.dev_pos_from_opt = True
                for point in meas.scanner_points:
                    point.device_moved()

            # meas.config['scanner_points_abs_glob_actual'] = False
            # meas.config['scanner_points_rel_glob_actual'] = False
            pos_last = meas.position

    def actualize_dev_pos_delta(self):
        # self.opt_vect_dev_actual = False
        # self.opt_vect_point_actual = False
        # self.opt_forces_actual = False
        pos_last = DevicePosition(mean_time_rel=0.0,
                                  dev_pos={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_speed={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_rot_vect={'X': 0.0, 'Y': 0.0, 'Z': 0.0},
                                  dev_euler_ang={'psi': 0.0, 'theta': 0.0, 'phi': 0.0})

        for meas in self.measurements:
            # if meas.config['abs_dev_pos_actual']:
            if meas.conf.available and meas.position.conf.pos_abs_actual:
                meas.position.set_position_deltas(previous_pos=pos_last)
                # meas.config['rel_dev_pos_actual'] = True
                meas.position.conf.pos_delta_actual = True
                pos_last = meas.position

    def actualize_points_glob(self):
        # self.opt_vect_dev_actual = False
        # self.opt_vect_point_actual = False
        # self.opt_forces_actual = False
        for meas in self.measurements:
            if meas.conf.available:
                tmp = False
                for point in meas.scanner_points:
                    if point.conf.pos_rel_actual:
                        if not point.conf.pos_abs_actual:
                            tmp = True
                            break
                if tmp:
                    meas.calc_glob_points_vect()
                    # meas.config['scanner_points_abs_glob_actual'] = True
                meas.position.conf.opt_loc_actual = False
                meas.position.conf.opt_force_actual = False

    def actualize_points_rel_glob(self):
        # self.opt_vect_dev_actual = False
        # self.opt_vect_point_actual = False
        # self.opt_forces_actual = False
        for meas in self.measurements:
            if meas.conf.available and meas.position.conf.pos_abs_actual:
                tmp = False
                for point in meas.scanner_points:
                    if not point.conf.pos_rel_actual:
                        tmp = True
                        break
                if tmp:
                    meas.calc_rel_glob_points_vect()

                if not tmp:
                    for point in meas.scanner_points:
                        if not point.conf.pos_abs_actual:
                            tmp = True
                            break
                if tmp:
                    meas.calc_glob_points_vect()

            meas.position.conf.opt_loc_actual = False
            meas.position.conf.opt_force_actual = False

    def voxelize(self, min_points, min_dimension, sample_in: float = 1.0, points_use=None):
        if sample_in is not None:
            if sample_in > 1.0:
                print('You cant choose more points than You have\nChanging to 1.0')
            if sample_in < 0.001:
                print('You cant choose less than 0.1% of points\nChanging to 0.001')
            sample_in = min(1.0, max(sample_in, 0.001))  # Use at least 0.1% of points

        # self.opt_vect_dev_actual = False
        # self.opt_vect_point_actual = False
        # self.opt_forces_actual = False
        # print('A')
        self.root_voxel = Voxel(min_points, min_dimension)
        # print('B')
        self.root_voxel.fill_root_voxel(self.measurements, sample_in=sample_in, points_use=points_use)
        # print('C')
        self.root_voxel.divide_voxel()
        # print('D')
        for meas in self.measurements:
            if meas.conf.available:
                meas.position.conf.opt_force_actual = False
                meas.position.conf.opt_loc_actual = False
                for point in meas.scanner_points:
                    point.conf.opt_vect_actual = False

    def count_nodes(self):
        self.total_nodes = self.root_voxel.count_nodes()
        return self.total_nodes

    def count_points(self):
        self.total_points = self.root_voxel.count_points()
        return self.total_points

    def export_voxels_wireframe(self):
        return self.root_voxel.get_subdivisions(0)

    def export_datastructure(self, get_all=False):
        # Make input datastructure back from out classes
        export_ds = []
        for meas in self.measurements:
            if meas.conf.available or get_all:
                rot_abs = meas.position.rot_abs.as_rotvec()
                export_ds.append({'time_rel': meas.position.mean_time_rel,
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
    device_curve.voxelize(min_points=100, min_dimension=0.1)
    total_nodes = device_curve.count_nodes()
    total_points = device_curve.count_points()
    print('Before')
    print('Nodes: ', total_nodes)
    print('Points: ', total_points)
    device_curve.root_voxel.drop_unneeded_nodes()
    total_nodes = device_curve.count_nodes()
    total_points = device_curve.count_points()
    print('After')
    print('Nodes: ', total_nodes)
    print('Points: ', total_points)
    print('Root_voxel_range: ', device_curve.root_voxel.range)
    print('Last position: ', device_curve.measurements[-1].position.pos_abs)
    # device_curve.apply_max_speed(0.2)
    device_curve.actualize_dev_pos_global()
    device_curve.actualize_points_glob()
    device_curve.voxelize(100, 0.1)
    export_data = device_curve.export_datastructure()
    print('Before')
    print('Nodes: ', total_nodes)
    print('Points: ', total_points)
    device_curve.root_voxel.drop_unneeded_nodes()
    total_nodes = device_curve.count_nodes()
    total_points = device_curve.count_points()
    print('After')
    print('Nodes: ', total_nodes)
    print('Points: ', total_points)
    print('Root_voxel_range: ', device_curve.root_voxel.range)
    print('Last position: ', device_curve.measurements[-1].position.pos_abs)

    print('filling surfaces')
    device_curve.root_voxel.fit_all_planes()

    planes = device_curve.root_voxel.get_planes()
    print(planes)
    # print('writing opt_vect')
    # device_curve.root_voxel.calc_points_opt_vect()
    # print('exporting ranges')
    # wire = device_curve.export_voxels_wireframe()

    # with open('/media/adamw/DATA/Projekty/Praca_mgr/scan_3d_MGR/gui_server/DataAnalysisClasses/test3.json',
    #           'w') as fout:
    #     json.dump(export_data, fout)
