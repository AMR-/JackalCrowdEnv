import numpy as np
import math
import copy


# Class to create a preprocessor that converts lidar input to polar occupancy grid
# It is assumed that the occupancy grid will span the same dimensions as the lidar
class Lidar2OccupancyGrid(object):
    def __init__(self,
                 lidar_min_distance=0.1/4.0,              # in meters
                 lidar_max_distance=4.0/4.0,              # in meters
                 # Our lidar is from -120 degrees to 120 degrees
                 lidar_radian_begin=-(2./3.)*math.pi,  # radian  beginning, measured with ahead as 0, clockwise positive
                 lidar_radian_end=(2./3.)*math.pi,     # radian  ending
                 lidar_arr_length=512,
                 occupancy_grid_number_of_columns=10,
                 occupancy_grid_row_heights=(0.2/4.0, 0.2/4.0, 0.2/4.0, 0.3/4.0, 1/4.0, 1/4.0)
                 ):
        self.lidar_min_distance = lidar_min_distance
        self.lidar_max_distance = lidar_max_distance
        self.lidar_radian_begin = lidar_radian_begin
        self.lidar_radian_end = lidar_radian_end
        self.radian_span = self.lidar_radian_end - self.lidar_radian_begin
        self.lidar_arr_length = lidar_arr_length
        self.lidar_radial_interval = self.radian_span / self.lidar_arr_length
        self.og_row_heights = occupancy_grid_row_heights
        self.og_columns = occupancy_grid_number_of_columns
        self.og_rows = len(occupancy_grid_row_heights) + 1
        self.grid_size_total = self.og_columns * self.og_rows
        self.column_width = self.radian_span / float(self.og_columns)
        self._bins = self._make_bins()
        self._r2c_map, self._column_list = self._create_radial_to_column_map()

    def lidar_to_occupancy_grid(self, lrange):
        assert len(lrange) == self.lidar_arr_length, \
            "got range: {} != required range {}".format(len(lrange), self.lidar_arr_length)
        rows = np.digitize(lrange, self._bins) - 1
        grid = np.zeros([self.og_rows, self.og_columns])
        for row, col, raw in zip(rows, self._column_list, lrange):
            if raw < self.lidar_max_distance:   # max distance would otherwise register as far away obstacle
                grid[row, col] = 1
        return grid

    def lidar_to_flattened_occupancy_grid(self, lrange):
        grid = self.lidar_to_occupancy_grid(lrange)
        return grid.flatten()

    def occupancy_grid_to_lidar(self, grid):
        # start_time = time.time()
        assert len(grid) == (self.og_rows * self.og_columns), \
            "grid has wrong shape {} instead of {}".format(len(grid), (self.og_rows * self.og_columns))
        grid = np.asarray(grid).reshape((self.og_rows, self.og_columns))

        lidar = np.ones((512,), dtype=float)
        for c in range(self.og_columns):
            for r in range(self.og_rows):
                if grid[r][c] == 1:
                    width = round(self.lidar_arr_length / self.og_columns)+1
                    if c == self.og_columns - 1:
                        end = self.lidar_arr_length
                    else:
                        end = (c+1) * width
                    if r == self.og_rows - 1:
                        r_end = 1.
                    else:
                        r_end = self._bins[r+1]
                    lidar[c*width:end] = (self._bins[r] + r_end) / 2
                    break
        return lidar

    def _make_bins(self):
        rdists = list(self.og_row_heights)
        rdists.insert(0, self.lidar_min_distance)
        bins = np.cumsum(rdists)
        return np.array(bins)

    def _create_radial_to_column_map(self):
        column_ct = 0
        radial_ct = 0
        column_traversed_dist = 0
        r2c_map = {}
        column_list = []
        while column_ct < self.lidar_arr_length:
            r2c_map[radial_ct] = column_ct
            column_list.append(column_ct)
            column_traversed_dist += self.lidar_radial_interval
            if column_traversed_dist > self.column_width:
                column_ct += 1
                column_traversed_dist = 0
        return r2c_map, np.array(column_list)


class SensorDataComplete(object):
    def __init__(self):
        self.lidar = []
        self.rgb = []
        self.depth = []
        self.goal = []
        self.velocity = []


class ObsFilter(object):
    def __init__(self, vel_thr=None, vel_expanded: bool = False):
        if vel_thr is None:
            vel_thr = [[0., 1.], [-1., 1.]]
        self.robot_sensors = SensorDataComplete()
        self.lidar_converter = Lidar2OccupancyGrid()
        self.vel_threshold = vel_thr
        self.vel_expanded = vel_expanded
        self.lidar_threshold = 4.
        # self.lidar_size = 70
        self.lidar_size: int = self.lidar_converter.grid_size_total
        self.number_of_actions: int = 10 if self.vel_expanded else 6

    def filter_lidar(self, lidar):
        if lidar.shape[0] != 512:
            lidar = np.where(np.isinf(lidar), self.lidar_threshold, lidar)
            lidar = np.where(np.isnan(lidar), self.lidar_threshold, lidar)
            lidar = np.where(lidar > self.lidar_threshold, self.lidar_threshold, lidar)
            lidar_processed = []
            rate = 720. / 512.
            threshold = rate
            for i in range(len(lidar)):
                if i > threshold:
                    lidar_processed.append(lidar[i])
                    threshold += rate
            lidar_processed.append(lidar[-1])
            return np.array(lidar_processed) / self.lidar_threshold
        else:
            return np.array(lidar) / self.lidar_threshold

    def lidar_filter(self, data):
        filtered_data = self.filter_lidar(data)
        filtered_data = self.lidar_converter.lidar_to_flattened_occupancy_grid(filtered_data)

        if len(self.robot_sensors.lidar) == 0:
            self.robot_sensors.lidar.append(copy.deepcopy(filtered_data))
            self.robot_sensors.lidar.append(copy.deepcopy(filtered_data))
            self.robot_sensors.lidar.append(copy.deepcopy(filtered_data))
        else:
            self.robot_sensors.lidar[2] = copy.deepcopy(self.robot_sensors.lidar[1])
            self.robot_sensors.lidar[1] = copy.deepcopy(self.robot_sensors.lidar[0])
            self.robot_sensors.lidar[0] = copy.deepcopy(filtered_data)

        lidar_data = np.array(self.robot_sensors.lidar)

        lidar_data = lidar_data.flatten()

        return lidar_data

    def lidar_umwrapper(self, data):
        now = self.lidar_converter.occupancy_grid_to_lidar(data[0:70])
        pre = self.lidar_converter.occupancy_grid_to_lidar(data[70:140])
        ppre = self.lidar_converter.occupancy_grid_to_lidar(data[140:])
        lidar = np.concatenate((np.asarray([ppre]).transpose(),
                                np.asarray([pre]).transpose(),
                                np.asarray([now]).transpose()), axis=1)
        return lidar

    def velocity_filter(self, data):
        filtered_data = np.array([self.velocity_wrapper(data)])
        return filtered_data

    @staticmethod
    def goal_filter(data):
        return np.array(data)

    def velocity_wrapper(self, vel):
        angular_range = []
        linear_range = self.vel_threshold[0][1] * 1. / 3.
        if self.vel_expanded:
            for i in range(1, 5, 1):
                angular_range.append(i * self.vel_threshold[1][1] / 5 - self.vel_threshold[1][1])

            if vel[0] > linear_range:
                if vel[1] < angular_range[0]:
                    return 0
                elif vel[1] < angular_range[1]:
                    return 6
                elif vel[1] < angular_range[2]:
                    return 2
                elif vel[1] < angular_range[3]:
                    return 8
                else:
                    return 4
            else:
                if vel[1] < angular_range[0]:
                    return 1
                elif vel[1] < angular_range[1]:
                    return 7
                elif vel[1] < angular_range[2]:
                    return 3
                elif vel[1] < angular_range[3]:
                    return 9
                else:
                    return 5
        else:
            for i in range(1, 3, 1):
                angular_range.append(i * self.vel_threshold[1][1] / 5 + -self.vel_threshold[1][1])
            if vel[0] > linear_range:
                if vel[1] < angular_range[0]:
                    return 0
                elif vel[1] < angular_range[1]:
                    return 2
                else:
                    return 4
            else:
                if vel[1] < angular_range[0]:
                    return 1
                elif vel[1] < angular_range[1]:
                    return 3
                else:
                    return 5

    def vel_2_simulator(self, data):
        filtered_data = self.velocity_unwrapper(data)
        filtered_data = np.array(filtered_data)
        return filtered_data

    def velocity_unwrapper(self, number):
        if not self.vel_expanded and number > 5:
            raise ValueError("Not using expanded action space, but received action larger than 5")
        action = [
            # Standard Action Space
            [self.vel_threshold[0][1], self.vel_threshold[1][0]],        # 0 (1,  -1)
            [self.vel_threshold[0][0], self.vel_threshold[1][0]],        # 1 (0,  -1)
            [self.vel_threshold[0][1], 0],                               # 2 (1,   0)
            [self.vel_threshold[0][0], 0],                               # 3 (0,   0)
            [self.vel_threshold[0][1], self.vel_threshold[1][1]],        # 4 (1,   1)
            [self.vel_threshold[0][0], self.vel_threshold[1][1]],        # 5 (0,   1)
            # Expanded Action Space (for oscillation fix)
            [self.vel_threshold[0][1] * 0.5, self.vel_threshold[1][0] * 0.5],  # 6 (0.5,  -0.5)
            [self.vel_threshold[0][0] * 0.5, self.vel_threshold[1][0] * 0.5],  # 7 (0,  -0.5)
            [self.vel_threshold[0][1] * 0.5, self.vel_threshold[1][1] * 0.5],  # 8 (0.5,   0.5)
            [self.vel_threshold[0][0] * 0.5, self.vel_threshold[1][1] * 0.5]   # 9 (0,   0.5)
        ][int(number)]
        return action
