import numpy as np
import tensorflow as tf
import tensorlayer as tl
from crowdenv.sensor_converter import ObsFilter


class NNModule:
    def __init__(self, path, continuous=False, vel_expanded=False, velocity_threshold=((0., 1.), (-1., 1.))):
        """
        :param velocity_threshold: [[linear_min, linear_max],[angular_min, angular_max]]
        """
        self.velocity_threshold = velocity_threshold

        self.sess = tf.InteractiveSession()
        self.scan_ph = tf.compat.v1.placeholder(tf.float32, [None, 512, 3], 'scan_ph')
        self.goal_ph = tf.compat.v1.placeholder(tf.float32, [None, 2], 'goal_ph')
        self.vel_ph = tf.compat.v1.placeholder(tf.float32, [None, 2], 'vel_ph')
        self.model, self.means = self._build_policy()
        self.value, self.quality = self._build_evaluation()
        self.continuous = continuous

        if not self.continuous:
            self.observation_filter = ObsFilter(self.velocity_threshold, vel_expanded)

        self.load(path)

    def _build_policy(self):
        net = tl.layers.InputLayer(self.scan_ph, name='scan_input')
        net = tl.layers.Conv1dLayer(net, act=tf.nn.relu, shape=[5, 3, 32], stride=2, name='cnn1')
        net = tl.layers.Conv1dLayer(net, act=tf.nn.relu, shape=[3, 32, 16], stride=2, name='cnn2')
        net = tl.layers.FlattenLayer(net, name='fl')
        net = tl.layers.DenseLayer(net, n_units=256, act=tf.nn.relu, name='cnn_output')
        cnn_output = net.outputs

        act_net = tl.layers.InputLayer(tf.concat([self.goal_ph, self.vel_ph, cnn_output], axis=1), name='nav_input')
        act_net = tl.layers.DenseLayer(act_net, n_units=128, act=tf.nn.relu, name='act1')
        linear = tl.layers.DenseLayer(act_net, n_units=1, act=tf.nn.sigmoid, name='linear')
        angular = tl.layers.DenseLayer(act_net, n_units=1, act=tf.nn.tanh, name='angular')
        with tf.compat.v1.variable_scope('means'):
            means = tf.concat([self.velocity_threshold[0][1] * linear.outputs,
                               self.velocity_threshold[1][1] * angular.outputs], axis=1)

        return [net, act_net, linear, angular], means

    def _build_evaluation(self):
        net = tl.layers.InputLayer(self.scan_ph, name='scan_value_input')
        net = tl.layers.Conv1dLayer(net, act=tf.nn.relu, shape=[5, 3, 32], stride=2, name='cnn1_value')
        net = tl.layers.Conv1dLayer(net, act=tf.nn.relu, shape=[3, 32, 16], stride=2, name='cnn2_value')
        net = tl.layers.FlattenLayer(net, name='fl_value')
        net = tl.layers.DenseLayer(net, n_units=256, act=tf.nn.relu, name='cnn_output_value')
        cnn_output = net.outputs

<<<<<<< HEAD
        act_net = tl.layers.InputLayer(tf.concat([self.goal_ph, self.vel_ph, cnn_output], axis=1), name='nav_input_value')
=======
        act_net = tl.layers.InputLayer(tf.concat([self.goal_ph, self.vel_ph, cnn_output], axis=1),
                                       name='nav_input_value')
>>>>>>> 25ad69f87fc0158bde48b4eddcbe599de49c5edb
        act_net = tl.layers.DenseLayer(act_net, n_units=128, act=tf.nn.relu, name='value')
        quality_net = tl.layers.DenseLayer(act_net, n_units=1, act=tf.nn.sigmoid, name='quality')
        quality = quality_net.outputs
        return [net, quality_net], quality

    def load(self, path):
        for i in range(len(self.model)):
<<<<<<< HEAD
            params = np.load(path+'model/best_act_{}.npz'.format(i), encoding="latin1", allow_pickle=True)['params']
            tl.files.assign_params(self.sess, params, self.model[i])
        for i in range(len(self.value)):
            params = np.load(path + 'model/best_val_{}.npz'.format(i), encoding="latin1", allow_pickle=True)['params']
=======
            params = np.load(path+'model/best_act_{}.npz'.format(i), encoding="latin1")['params']
            tl.files.assign_params(self.sess, params, self.model[i])
        for i in range(len(self.value)):
            params = np.load(path + 'model/best_val_{}.npz'.format(i), encoding="latin1")['params']
>>>>>>> 25ad69f87fc0158bde48b4eddcbe599de49c5edb
            tl.files.assign_params(self.sess, params, self.value[i])

    def predict_q(self, obs):
        if not self.continuous:
            lidar = self.observation_filter.lidar_umwrapper(obs[3:])
            action = self.observation_filter.velocity_unwrapper(obs[2])
            goal = obs[0:2]
            obs = (lidar, goal, action)

        feed_dict = {
            self.scan_ph: [obs[0]],
            self.goal_ph: [obs[1]],
            self.vel_ph: [obs[2]]
        }

        quality = self.sess.run(self.quality, feed_dict=feed_dict)[0]
        return quality

    def predict(self, obs):
        if not self.continuous:
            lidar = self.observation_filter.lidar_umwrapper(obs[3:])
            action = self.observation_filter.velocity_unwrapper(obs[2])
            goal = obs[0:2]
            obs = (lidar, goal, action)

        feed_dict = {
            self.scan_ph: [obs[0]],
            self.goal_ph: [obs[1]],
            self.vel_ph: [obs[2]]
        }

        action = self.sess.run(self.means, feed_dict=feed_dict)[0]

        if action[0] < self.velocity_threshold[0][0]:
            action[0] = self.velocity_threshold[0][0]
        elif action[0] > self.velocity_threshold[0][1]:
            action[0] = self.velocity_threshold[0][1]

        if action[1] < self.velocity_threshold[1][0]:
            action[1] = self.velocity_threshold[1][0]
        elif action[1] > self.velocity_threshold[1][1]:
            action[1] = self.velocity_threshold[1][1]

        final_action = self.observation_filter.velocity_wrapper(action)

        return final_action
<<<<<<< HEAD

if __name__ == "__main__":
    i=1
    params = np.load('model/best_val_{}.npz'.format(i), encoding="latin1")['params']

    print("test")

=======
>>>>>>> 25ad69f87fc0158bde48b4eddcbe599de49c5edb
