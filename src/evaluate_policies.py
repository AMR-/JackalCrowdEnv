import rospy
import math
import numpy as np

from crowdenv.rl import CrowdENV
from crowdenv.networks import NNModule


def get_distance(pose1, pose2):
    return math.sqrt(abs(pose1[0] - pose2[0]) ** 2 + abs(pose1[1] - pose2[1]) ** 2)


def evaluate_trajectories(env: CrowdENV, number_of_trials: int, vel_expanded=False):
    env.reset()
    policy = NNModule(path="./crowdenv/",
                      continuous=False,
                      vel_expanded=vel_expanded)

    value = 0
    obs, reward, done, info = env.reset()
    step_number = 0

    start_time = rospy.get_rostime().to_sec()
    collision = 0
    success = 0
    freezing = 0
    last_position = info["position"]
    trajectory_length = 0
    success_trajectory = []
    all_trajectory = []
    run_time = []
    success_time = []
    steps = []
    success_steps = []
    success_rewards = []
    total_rewards = []
    cumulated_reward = reward
    oscillations = []

    while (not rospy.is_shutdown()) and (value < number_of_trials):
        if done:
            total_time = (rospy.get_rostime().to_sec() - start_time)
            value += 1

            if info["success"]:
                success += 1
                success_trajectory.append(trajectory_length)
                success_time.append(total_time)
                success_steps.append(step_number)
                success_rewards.append(cumulated_reward)
            elif info["collision"]:
                collision += 1
            else:
                freezing += 1

            all_trajectory.append(trajectory_length)
            run_time.append(total_time)
            steps.append(step_number)
            total_rewards.append(cumulated_reward)
            oscillations.append(info["total_oscillation"])

            obs, reward, done, info = env.reset()
            trajectory_length = 0
            last_position = info["position"]
            start_time = rospy.get_rostime().to_sec()
            step_number = 0
            cumulated_reward = reward
            continue

        action = policy.predict(obs)

        obs, reward, done, info = env.step(action)
        cumulated_reward += reward

        step_number += 1
        trajectory_length += get_distance(last_position, info["position"])

        last_position = info["position"]

    return {"total_iterations": number_of_trials,
            "success": success,
            "collision": collision,
            "freezing": freezing,
            "trajectory_length": all_trajectory,
            "successful_trajectory_length": success_trajectory,
            "run_time": run_time,
            "successful_run_time": success_time,
            "steps": steps,
            "successful_steps": success_steps,
            "total_reward": total_rewards,
            "successful_rewards": success_rewards,
            "oscillation": oscillations}


def run_evaluation(scenarios, seed, iterations=20, max_timesteps=200, vel_expanded=False):
    for sce in scenarios:
        env = CrowdENV(scenarios_index=sce,
                       max_steps=max_timesteps,
                       random_seed=seed,
                       vel_expanded=vel_expanded,
                       target_threshold=0.8)
        result = evaluate_trajectories(env, iterations, vel_expanded)
        deal_with_results(result)


def deal_with_results(result):
    print("policy: {}".format(result["policy"]))
    print("average rewards per ts: {}".format(np.sum(np.asarray(result["total_reward"])) / np.sum(result["steps"])))
    print("average rewards: {}".format(np.mean(np.asarray(result["total_reward"]))))
    print("crashes: {}".format(result["collision"]))
    print("success: {}".format(result["success"]))
    print("freezing: {}".format(result["freezing"]))
    print("osicllation: {}".format(np.mean(np.asarray(result["oscillation"]))))
    print("trajectory  length: {}".format(np.mean(np.asarray(result["trajectory_length"]))))
    print("successful trajectory  length: {}".format(np.mean(np.asarray(result["successful_trajectory_length"]))))
    print("run time: {}".format(np.mean(np.asarray(result["run_time"]))))
    print("successful run time: {}".format(np.mean(np.asarray(result["successful_run_time"]))))


if __name__ == "__main__":
    rospy.init_node('robot', anonymous=True)
    scenarios = [11]
    for i in scenarios:
        run_evaluation(scenarios, seed=0, iterations=50, max_timesteps=100, vel_expanded=False)
