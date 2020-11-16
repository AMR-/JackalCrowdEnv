from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
import rospy
import random
import math
import numpy as np
import copy


class DisplayConfig(object):
    agent = "agent"
    path = "path"
    goal = "goal"
    agent_topic = "agent_markers"
    goal_topic = "goal_markers"
    path_topic = "path_markers"

    agent_size = [0.24, 0.24, 0.01]
    agent_color = [.0, 0.12, 1., 1.]
    agent_text_color = [1., 1., 1., 1.]
    agent_text_size = [0., 0., 0.02]

    goal_color = [1., 1., 0., .8]
    goal_size = [.3, .3, .01]
    goal_text_color = [0., 0.5, 0.5, .7]
    goal_text_size = [.0, .0, .20]

    path_size = [0.1, 0., 0.]


class Visualization(object):
    def __init__(self, robot_number=1):
        self.robot_number = robot_number
        self.marker_config = DisplayConfig()

        self.agent_colors = self._get_color()

        self.path_markers = MarkerArray()
        self.path_markers_id = 0
        self._initialize_array(ns=DisplayConfig.path)
        self.path_markers_pub = rospy.Publisher(DisplayConfig.path_topic, MarkerArray, queue_size=10)

        self.goal_markers = MarkerArray()
        self.goal_markers_id = 0
        self._initialize_array(ns=DisplayConfig.goal)
        self.goal_markers_pub = rospy.Publisher(DisplayConfig.goal_topic, MarkerArray, queue_size=10)

        self.agent_markers = MarkerArray()
        self.agent_markers_id = 0
        self._initialize_array(ns=DisplayConfig.agent)
        self.agent_markers_pub = rospy.Publisher(DisplayConfig.agent_topic, MarkerArray, queue_size=10)
        self.agent_last_poses = None

    def _initialize_array(self, ns=None):
        for i in range(self.robot_number):
            if ns == DisplayConfig.agent:
                self._create_marker(marker_type=Marker.CYLINDER,
                                    color=self.marker_config.agent_color,
                                    scale=self.marker_config.agent_size, ns=ns, text=None)
                self._create_marker(marker_type=Marker.TEXT_VIEW_FACING,
                                    color=self.marker_config.agent_text_color,
                                    scale=self.marker_config.agent_text_size, ns=ns, text=str(i))
            elif ns == DisplayConfig.path:
                self._create_marker(marker_type=Marker.LINE_STRIP,
                                    color=[self.agent_colors[i][0], self.agent_colors[i][1],
                                           self.agent_colors[i][2], 0.1],
                                    scale=self.marker_config.path_size, ns=ns)
            elif ns == DisplayConfig.goal:
                self._create_marker(marker_type=Marker.CYLINDER,
                                    color=self.marker_config.goal_color,
                                    scale=self.marker_config.goal_size, ns=ns, text=None)
                self._create_marker(marker_type=Marker.TEXT_VIEW_FACING,
                                    color=self.marker_config.goal_text_color,
                                    scale=self.marker_config.goal_text_size, ns=ns, text=str(i))

    def reset(self):
        self.path_markers = MarkerArray()
        self.path_markers_id = 0
        self._initialize_array(ns=DisplayConfig.path)

        self.goal_markers = MarkerArray()
        self.goal_markers_id = 0
        self._initialize_array(ns=DisplayConfig.goal)

        self.agent_markers = MarkerArray()
        self.agent_markers_id = 0
        self._initialize_array(ns=DisplayConfig.agent)
        self.agent_last_poses = None

    def display(self, agent_positions, goal_positions, time_step):
        self._show_agent(agent_positions)
        self._show_goal(goal_positions)
        self._show_path(agent_positions, time_step)

    def _get_color(self):
        colors = []
        random.seed(1)
        color_numbers = random.sample(range(6 * 6 * 6), self.robot_number)

        for i in range(self.robot_number):
            r = math.floor(color_numbers[i] / 36)
            color_numbers[i] -= r * 36
            g = math.floor(color_numbers[i] / 6)
            b = color_numbers[i] - g * 6
            colors.append([r, g, b])
        colors = np.array(colors) * 51 / 255
        return colors

    def _show_agent(self, positions):
        for i in range(len(positions)):
            self._update_marker(ns=DisplayConfig.agent, index=i, position=positions[i])
        self.agent_markers_pub.publish(self.agent_markers)

    def _show_path(self, positions, step_number):
        if self.agent_last_poses is None:
            self.agent_last_poses = positions
        for i in range(len(positions)):
            self._update_marker(ns=DisplayConfig.path, index=i, position=positions[i], color_increase=step_number / 200)
        self.agent_last_poses = copy.deepcopy(positions)
        self.path_markers_pub.publish(self.path_markers)

    def _show_goal(self, positions):
        for i in range(len(positions)):
            self._update_marker(ns=DisplayConfig.goal, index=i, position=positions[i])
        self.goal_markers_pub.publish(self.goal_markers)

    def _update_marker(self, ns, index, position=None, color_increase=None):
        if ns == DisplayConfig.agent:
            self.agent_markers.markers[index * 2].header.stamp = rospy.Time.now()
            self.agent_markers.markers[index * 2 + 1].header.stamp = rospy.Time.now()
            self.agent_markers.markers[index * 2].pose = position
            self.agent_markers.markers[index * 2 + 1].pose = position
            self.agent_markers.markers[index * 2 + 1].pose.position.y += 0.2
        elif ns == DisplayConfig.path:
            self.path_markers.markers[index].header.stamp = rospy.Time.now()
            self.path_markers.markers[index].points.append(position.position)
            self.path_markers.markers[index].color.a += color_increase
        elif ns == DisplayConfig.goal:
            self.goal_markers.markers[index * 2].header.stamp = rospy.Time.now()
            self.goal_markers.markers[index * 2 + 1].header.stamp = rospy.Time.now()
            self.goal_markers.markers[index * 2].pose = position
            self.goal_markers.markers[index * 2 + 1].pose = position
            self.goal_markers.markers[index * 2 + 1].pose.position.y += 0.2

    def _create_marker(self, marker_type, color, scale, ns=None, text=None, position=None):
        marker = Marker()
        marker.header.frame_id = 'ground_truth'
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        if text:
            marker.text = text
            marker.pose.position.z += 0.1
            if ns == DisplayConfig.goal:
                marker.pose.position.y += 0.2
        if ns == DisplayConfig.agent:
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = "file://%s" % rospy.get_param("gazebo/dae_file")
            marker.id = self.agent_markers_id
            self.agent_markers.markers.append(marker)
            self.agent_markers_id += 1
        elif ns == DisplayConfig.path:
            marker.id = self.path_markers_id
            self.path_markers.markers.append(marker)
            self.path_markers_id += 1
        elif ns == DisplayConfig.goal:
            marker.id = self.goal_markers_id
            self.goal_markers.markers.append(marker)
            self.goal_markers_id += 1