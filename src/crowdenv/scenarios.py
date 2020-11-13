import random
import numpy as np


class Scenarios:
    def __init__(self, seed):
        self.starts = None
        self.goals = None

        self.obstacles = self._get_obstalces()
        self.radius = 0.5
        random.seed(seed)

    def choose_scenarios(self, scenarios_index):
        scenarios = {
            0: self.empty_ur(2),  # empty with fixed start and goal locations
            1: self.empty_dr(2),
            2: self.empty_sf(2),
            3: self.empty_sb(2),

            4: self.empty_ur(),  # empty with fixed start and goal locations
            5: self.empty_dr(),
            6: self.empty_sf(),
            7: self.empty_sb(),

            8: self.empty_rand_d(),
            9: self.empty_rand_u(),
            10: self.cross_left(),
            11: self.cross_right(),

            12: self.diomand_left(),
            13: self.diomand_right(),
            14: self.barriers_d(),
            15: self.barriers_u(),

            16: self.static_humnan_rand(),
            17: self.static_obstacles_rand(),
            18: self.dynamic_l(),
            19: self.dynamic_r()
        }
        return scenarios[scenarios_index]

    def reset(self):
        self.starts = []
        self.goals = []

    def _get_obstalces(self):
        # x, y, length, width
        return [
            # standing human
            [4.2431, 5.94947, 0.5, 0.35],
            [5., 8., 0.5, 0.35],
            [2.14243, 8.16216, 0.5, 0.35],
            [0.614771, 6.90434, 0.5, 0.35],
            [-1.11476, 5.28075, 0.5, 0.35],
            [0.252548, 3.63599, 0.5, 0.35],
            [6.5698, 2.75425, 0.5, 0.35],
            [7.69805, 5.47317, 0.5, 0.35],
            [8.28105, 8.85375, 0.5, 0.35],
            [6.19488, 13.4037, 0.5, 0.35],
            [2.80411, 12.9829, 0.5, 0.35],
            [0.608639, 12.346, 0.5, 0.35],
            [-0.797044, 10.1889, 0.5, 0.35],
            [-2.01031, 8.1019, 0.5, 0.35],
            [-2.5795, 14.1336, 0.5, 0.35],
            [-2.53658, 12.3419, 0.5, 0.35],
            [0.643331, 14.6162, 0.5, 0.35],
            [4.60959, 11.0017, 0.5, 0.35],
            [9.17129, 12.1975, 0.5, 0.35],
            [9.81664, 2.95017, 0.5, 0.35],
            [-2.59728, 2.44068, 0.5, 0.35],
            [3.73534, 5.65789, 0.35, 0.5],

            # static obstacles
            self._xy_to_xylw([-2.5, 1., -4., -1.]),
            self._xy_to_xylw([2.8, 3.2, -1.5, 1.]),
            self._xy_to_xylw([5.5, 9., -4.5, -1.5]),
            self._xy_to_xylw([8.5, 12, -5., -4.2]),
            self._xy_to_xylw([-3., 0., -11.5, -8.5]),
            self._xy_to_xylw([0., 2., -8., -5.5]),
            self._xy_to_xylw([2., 3., -7.5, -4.5]),
            self._xy_to_xylw([3., 4., -6.5, -4.]),
            self._xy_to_xylw([4., 5., -6., -4.]),
            self._xy_to_xylw([1.5, 5., -13., -9.5]),
            self._xy_to_xylw([6., 10., -10.5, -6.5])
            ]

    def _xy_to_xylw(self, data):
        l = data[1] - data[0]
        w = data[3] - data[2]
        x = l / 2. + data[0]
        y = w / 2. + data[2]
        return [x, y, l, w]

    def test_location(self, x, y, radius):
        success = True
        for obs in self.obstacles:
            # obs: x,y,w,h
            left = obs[0] - obs[2] / 2. - radius
            right = obs[0] + obs[2] / 2. + radius
            up = obs[1] + obs[3] / 2. + radius
            down = obs[1] - obs[3] / 2. - radius
            if right > x > left and up > y > down:
                success = False
        return success

    def generate_position(self, radius, start_position=[0., 0.], environ_size=[5., 5.]):
        success_set = False
        x, y = 0.0, 0.0
        while not success_set:
            x = random.random() * environ_size[0] + start_position[0]
            y = random.random() * environ_size[1] + start_position[1]
            success_set = self.test_location(x, y, radius=radius)
        a = 2. * np.pi * random.uniform(0., 1.) - np.pi
        return [x, y, a]

    def empty_rand_d(self):
        sx, sy, sa = 60., 10., random.uniform(0., 1.) * 6.28 - 3.14
        gx, gy, ga = sx + 5. * random.uniform(-1., 1.), sy + 5 * random.uniform(-1., 1.), 0.
        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def empty_rand_u(self):
        sx, sy, sa = 60., -10., random.uniform(0., 1.) * 6.28 - 3.14
        gx, gy, ga = sx + 5. * random.uniform(-1., 1.), sy + 5 * random.uniform(-1., 1.), 0.
        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def empty_ur(self, distance=5.0):
        sx, sy, sa = 40., 20., 0.
        gx, gy, ga = sx + distance, sy + distance, 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def empty_dr(self, distance=5.0):
        sx, sy, sa = 40., 10., 0.
        gx, gy, ga = sx + distance, sy - distance, 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def empty_sf(self, distance=5.0):
        sx, sy, sa = 40., 0., 0.
        gx, gy, ga = sx + distance, sy, 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def empty_sb(self, distance=5.0):
        sx, sy, sa = 40., -10., 0.
        gx, gy, ga = sx - distance, sy, 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def static_humnan_rand(self):
        sx, sy, sa = self.generate_position(self.radius, [-3., 2.5], [12, 12])
        gx, gy, ga = self.generate_position(self.radius, [-3., 2.5], [12, 12])

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def static_obstacles_rand(self, ):
        sx, sy, sa = self.generate_position(self.radius, [-3., -12.5], [12, 12])
        gx, gy, ga = self.generate_position(self.radius, [-3., -12.5], [12, 12])

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def cross_left(self):
        sx, sy, sa = -14., -11., np.pi/2.
        gx, gy, ga = -14., -3., np.pi/2.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def cross_right(self):
        #sx, sy, sa = -8., -9., np.pi / 2.
        sx, sy, sa = -9., -9, np.pi / 2.
        gx, gy, ga = -8., -3., np.pi / 2.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def diomand_left(self):
        sx, sy, sa = -12., 5., np.pi/2.
        gx, gy, ga = -12., 12., np.pi/2.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def diomand_right(self):
        sx, sy, sa = -8., 5., np.pi/2.
        gx, gy, ga = -8., 12., np.pi/2.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def barriers_d(self):
        sx, sy, sa = -25, -10, np.pi
        gx, gy, ga = -25, 0, 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def barriers_u(self):
        sx, sy, sa = -22., 1., np.pi
        gx, gy, ga = -22., 14.5, 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def dynamic_l(self):
        sx, sy, sa = 13., 4., 0.
        gx, gy, ga = 26., 5., 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals

    def dynamic_r(self):
        sx, sy, sa = 13., 11., 0.
        gx, gy, ga = 26., 10., 0.

        self.starts = [sx, sy, sa]
        self.goals = [gx, gy, ga]
        return self.starts, self.goals
