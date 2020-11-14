if __name__ == "__main__":
    import sys

    sys.path.append("../")

import rospy
import time
import math
import copy
import threading
import random
import numpy as np
from abc import abstractmethod, ABC

from gym import Env, spaces
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from crowdenv.networks import NNModule
from crowdenv.scenarios import Scenarios
from crowdenv.sensor_converter import ObsFilter
from crowdenv.oscillation_detector import OscillationDetect
from crowdenv.display import Visualization

class TOPICS(object):
    def __init__(self):
        self.scan_topic = "/front/scan"
        self.reset_service = '/gazebo/reset_world'
        self.set_service = '/gazebo/set_model_state'
        self.states_topic = "/gazebo/model_states"

        self.odom_topic = "odometry/filtered"
        self.action_topic = "/jackal_velocity_controller/cmd_vel"


class HasActionObservationSpace(ABC):

    @property
    @abstractmethod
    def observation_space(self) -> spaces.Space:
        raise NotImplementedError()

    @property
    @abstractmethod
    def action_space(self) -> spaces.Space:
        raise NotImplementedError()


class CrowdENV(Env, HasActionObservationSpace):
    def __init__(self,
                 scenarios_index=0,
                 collision_threshold=0.22,
                 target_threshold=0.2,
                 step_time=0.1,
                 max_steps=500,
                 random_seed=1,
                 vel_expanded: bool = False,  # whether to use the OSC expanded Action Space or not
                 ):
        self.seed = random_seed
        self.robot_name = "jackal"
        self.collision_threshold = collision_threshold
        self.target_threshold = target_threshold
        self.start = None
        self.goal = None
        self.step_time = step_time
        self.max_steps = max_steps

        self.display = Visualization(1)

        self.vel_threshold = ((0., 1.), (-1., 1.))
        self.vel_expanded = vel_expanded

        self.scenarios = Scenarios(self.seed)
        self.scenarios_index = scenarios_index

        self.topics = TOPICS()
        self._generate_advertisements()

        self.observation_filter = ObsFilter(vel_thr=self.vel_threshold,
                                            vel_expanded=self.vel_expanded)
        cells_per_occupancy_grid: int = self.observation_filter.lidar_converter.grid_size_total
        num_timesteps: int = 3
        self.observation_space_ = spaces.Box(
            np.array([-math.pi, 0] + [0]
                     + [0] * (num_timesteps * cells_per_occupancy_grid)),
            np.array([math.pi, 4] + [self.observation_filter.number_of_actions - 1]
                     + [1] * (num_timesteps * cells_per_occupancy_grid))
        )
        self.action_space_ = spaces.Discrete(self.observation_filter.number_of_actions)
        print("DEBUG: action space, observation space")
        print(self.action_space)
        print(self.observation_space)
        # print("crowdenv: {}".format(self.vel_expanded))
        self.trajectory = []

        self.position = None
        self.velocity = None
        self.action = None
        self.reward = None
        self.scan = None
        self.scan_single = None
        self.rel_goal = None
        self.last_rel_goal = None

        self.terminatec = False
        self.terminateg = False
        self.terminates = False

        self.last_time = rospy.get_rostime().to_sec()
        self.step_size = 0

        self.oscillation_detector = OscillationDetect()
        self.total_oscillations = 0

        # self.reset()

    @property
    def observation_space(self) -> spaces.Space:
        return self.observation_space_

    @property
    def action_space(self) -> spaces.Space:
        return self.action_space_

    def reset(self, scenarios_index=None):
        if scenarios_index is not None:
            self.scenarios_index = scenarios_index
        self.start, self.goal = self.scenarios.choose_scenarios(self.scenarios_index)
        self._set_robot_srv()

        self.position = None
        self.velocity = None
        self.action = None
        self.reward = None
        self.scan = None
        self.scan_single = None
        self.rel_goal = None
        self.last_rel_goal = None

        self.terminatec = False
        self.terminateg = False
        self.terminates = False

        self.last_time = rospy.get_rostime().to_sec()
        self.step_size = 0

        self.total_oscillations = 0
        self.display.reset()

        while (self.position is None) or (self.scan_single is None) or (self.velocity is None):
            pass
        return self.get_observation()

    def render(self, mode='human'):
        pass

    def _set_robot_srv(self):
        state_msg = ModelState()
        state_msg.model_name = self.robot_name
        state_msg.pose.position.x = self.start[0]
        state_msg.pose.position.y = self.start[1]
        state_msg.pose.orientation.z = np.sin(self.start[2] / 2)
        state_msg.pose.orientation.w = np.cos(self.start[2] / 2)

        rospy.wait_for_service(self.topics.set_service)
        try:
            set_robot = rospy.ServiceProxy(self.topics.set_service, SetModelState)
            set_robot(state_msg)
        except rospy.ServiceException as err:
            print("Service call failed: {}".format(err))
        rospy.sleep(0.5)

    def _generate_advertisements(self):
        self.scan_subscriber = rospy.Subscriber(self.topics.scan_topic, LaserScan, self._scan_callback, queue_size=1)

        self.publisher = rospy.Publisher(self.topics.action_topic, Twist, queue_size=10)

        self.odom_subscriber = rospy.Subscriber(self.topics.odom_topic, Odometry, self._odom_callback, queue_size=1)

        self.pose_subscriber = rospy.Subscriber(self.topics.states_topic, ModelStates, self._state_callback,
                                                queue_size=1)

    def _scan_callback(self, data):
        self.scan_single = np.asarray([data.ranges]).transpose()
        # print("max:{}/	min:{}".format(np.max(scan), np.min(scan)))

        min_dis_collision = self.scan_single.min()
        if min_dis_collision < self.collision_threshold:
            self.terminatec = True
            self.publisher.publish(Twist())

    def _odom_callback(self, data):
        self.velocity = np.array([data.twist.twist.linear.x, data.twist.twist.angular.z])

    def _state_callback(self, states):
        for i in range(len(states.name)):
            if states.name[i] == self.robot_name:
                self.position = states.pose[i]

    def _calculate_rel_goal(self):
        robot_rel_y = self.goal[1] - self.position.position.y
        robot_rel_x = self.goal[0] - self.position.position.x
        distance = math.hypot(robot_rel_x, robot_rel_y)

        sin_y = 2 * (self.position.orientation.w * self.position.orientation.z +
                     self.position.orientation.x * self.position.orientation.y)
        cos_y = 1 - 2 * (self.position.orientation.y ** 2 + self.position.orientation.z ** 2)
        orientation = np.arctan2(robot_rel_y, robot_rel_x) - np.arctan2(sin_y, cos_y)
        if orientation > np.pi:
            orientation -= 2 * np.pi
        elif orientation < -np.pi:
            orientation += 2 * np.pi
        else:
            orientation = orientation
        return np.array([distance, orientation])

    def _calculus_reward(self):
        reward = 0

        self.rel_goal = self._calculate_rel_goal()
        if self.last_rel_goal is None:
            self.last_rel_goal = copy.deepcopy(self.rel_goal)

        if self.rel_goal[0] < self.target_threshold:
            self.terminateg = True
            reward += 20.
        elif self.terminatec:
            reward -= 20.
        else:
            reward = 0.2 * (self.last_rel_goal[0] - self.rel_goal[0])
        self.last_rel_goal = copy.deepcopy(self.rel_goal)
        return reward

    def step(self, action, continuous=False):
        if self.oscillation_detector.recognise(action):
            self.total_oscillations += 1
            # print("oscillation", end=": ")
        if continuous:
            action_final = action
        else:
            action_final = self.observation_filter.velocity_unwrapper(action)
        # print("{}/{}".format(action_final[0], action_final[1]))

        velocity = Twist()
        velocity.linear.x = action_final[0]
        velocity.angular.z = action_final[1]

        while not (self.publisher.get_num_connections() > 0):
            pass
        self.publisher.publish(velocity)
        self.step_size += 1
        if self.step_size >= self.max_steps:
            self.terminates = True

        while rospy.get_rostime().to_sec() - self.last_time < self.step_time:
            pass
        self.last_time = rospy.get_rostime().to_sec()

        obs, reward, done, info = self.get_observation()

        # print("goal: {}; position: {}, collision:{}; terminate: {}; time_step_up:{}".format([self.goal[0], self.goal[1]],
        #                                                                    [self.position.position.x, self.position.position.y],
        #                                                                    self.terminatec, self.terminateg, self.terminates))

        goal_pose = Pose()
        goal_pose.position.x = self.goal[0]
        goal_pose.position.y = self.goal[1]
        self.display.display([self.position], [goal_pose], self.step_size)
        self.trajectory.append([obs,
                                action,
                                reward,
                                self.step_size,
                                self.position,
                                rospy.get_rostime().to_sec(),
                                (self.terminatec, self.terminateg)])
        return obs, reward, done, info

    def get_terminate(self):
        return self.terminatec or self.terminateg or self.terminates

    def get_observation(self):
        reward = self._calculus_reward()
        processed_lidar = self.observation_filter.lidar_filter(self.scan_single)
        processed_vel = self.observation_filter.velocity_filter(self.velocity)
        processed_goal = self.observation_filter.goal_filter(self.rel_goal)
        total = np.concatenate((processed_goal, processed_vel, processed_lidar), axis=0)

        yaw = math.atan2(2 * (self.position.orientation.x * self.position.orientation.y +
                              self.position.orientation.w * self.position.orientation.z),
                         self.position.orientation.w * self.position.orientation.w +
                         self.position.orientation.x * self.position.orientation.x -
                         self.position.orientation.y * self.position.orientation.y -
                         self.position.orientation.z * self.position.orientation.z)
        return total, reward, self.get_terminate(), {"collision": self.terminatec,
                                                     "success": self.terminateg,
                                                     "position": [self.position.position.x, self.position.position.y,
                                                                  yaw],
                                                     "total_oscillation": self.total_oscillations,
                                                     "lidar": self.scan_single}

#
# class TrajectoryGenerator:
#     def __init__(self, iterations=50):
#         self.env = CrowdENV(scenarios_index=10, max_steps=100)
#         self.policy = NNModule(velocity_threshold=self.env.vel_threshold, path="./", continuous=False)
#         self.iterations = iterations
#
#     def run(self, scenarios=range(0, 20, 1)):
#         for i in scenarios:
#             if i < 8:
#                 self.env.max_steps = 300
#             elif i < 10:
#                 self.env.max_steps = 300
#             elif i < 14:
#                 self.env.max_steps = 400
#             elif i < 18:
#                 self.env.max_steps = 500
#             else:
#                 self.env.max_steps = 800
#             self.get_trajectory(i)
#
#     def get_trajectory(self, scenario):
#         # for i in range(steps):
#         value = 0
#         total_trajectories = []
#
#         obs, reward, done, _ = self.env.reset(scenario)
#
#         while (not rospy.is_shutdown()) and (value < self.iterations):
#
#             if done:
#                 value += 1
#                 total_trajectories.append(copy.deepcopy(self.env.trajectory))
#                 print((self.env.terminateg, self.env.terminatec), end=" ")
#                 obs, reward, done, _ = self.env.reset(scenario)
#                 print("obs: {}, reward: {}, done: {} ".format(obs.shape, reward,
#                                                               (self.env.terminateg, self.env.terminatec)))
#
#             action = self.policy.step(obs)
#             obs, reward, done, _ = self.env.step(action)
#             print("{},{},{}".format(self.env.step_size, reward, done))
#
#         np.save("trajectory_{}".format(scenario), np.asarray(total_trajectories))
#
#
# def run_iterations():
#     tg = TrajectoryGenerator(2)
#     tg.run([17])
#
#
# if __name__ == "__main__":
#     rospy.init_node('robot', anonymous=True)
#
#     t1 = threading.Thread(target=rospy.spin)
#     t2 = threading.Thread(target=run_iterations)
#
#     t1.start()
#     t2.start()
#
#     t2.join()
#     t1.join()
