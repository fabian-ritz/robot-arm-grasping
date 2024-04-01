import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from string_log_channel import StringLogChannel
from tensorboard_callback import TensorboardCallback

MODE = "eval"  # train, eval, demo
ENV_NAME = "lift5d"

TRAIN_STEPS = 6000000
CHECKPOINT_STEPS = int(TRAIN_STEPS/60)
EVAL_EPISODES = 100
DEMO_EPISODES = 50

config_channel = EngineConfigurationChannel()
log_channel = StringLogChannel()

# Custom metrics in tensorboard
logging_callback = TensorboardCallback(log_channel)

# Save a checkpoint every 100000 steps
checkpoint_callback = CheckpointCallback(
  save_freq=CHECKPOINT_STEPS,
  save_path="./models/checkpoints",
  name_prefix=f"sac_{ENV_NAME}_{TRAIN_STEPS}",
  save_replay_buffer=True,
  save_vecnormalize=True
)

if MODE == "train":
    unity_env = UnityEnvironment(f"./{ENV_NAME}", no_graphics=True,
                                 side_channels=[config_channel, log_channel])
    env = UnityToGymWrapper(unity_env, allow_multiple_obs=False)
    config_channel.set_configuration_parameters(time_scale=10.0)

    model = SAC("MlpPolicy",
                env=env,
                learning_starts=2000,
                buffer_size=200000,
                train_freq=16,
                batch_size=256,
                verbose=1,
                tensorboard_log=f"./tensorboard/sac_{ENV_NAME}_{TRAIN_STEPS}")
    model.learn(TRAIN_STEPS, callback=[logging_callback, checkpoint_callback])
    model.save(f"models/sac_{ENV_NAME}_{TRAIN_STEPS}")
    env.close()

if MODE == "eval":
    unity_env = UnityEnvironment(f"./{ENV_NAME}", no_graphics=True,
                                 side_channels=[config_channel, log_channel])
    env = UnityToGymWrapper(unity_env, allow_multiple_obs=False)
    config_channel.set_configuration_parameters(time_scale=10.0)
    model = SAC.load(f"models/sac_{ENV_NAME}_{TRAIN_STEPS}", env=env)
    (all_successes, all_scores, all_oob, all_grips, all_collisions, all_target_distances, all_goal_distances,
     all_returns, all_steps) = ([], [], [], [], [], [], [], [], [])
    for i in range(EVAL_EPISODES):
        obs = env.reset()
        G = 0
        step = 0
        done = False
        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, _info = env.step(action)
            G += (0.99 ** step) * reward
            step += 1
        all_returns.append(G)
        all_steps.append(step)
        ep_results = log_channel.lastMessage.replace(",", ".").split(";")
        all_successes.append(int(ep_results[0]))
        all_oob.append(int(ep_results[1]))
        all_scores.append(float(ep_results[2]))
        all_grips.append(int(ep_results[3]))
        all_collisions.append(int(ep_results[4]))
        all_target_distances.append(float(ep_results[5]))
        all_goal_distances.append(float(ep_results[6]))
    print("Success rate:     " + str(np.round(np.average(all_successes), decimals=2)) +
          "\navg. target oob:  " + str(np.round(np.average(all_oob), decimals=2)) +
          "\navg. score:       " + str(np.round(np.average(all_scores), decimals=2)) +
          "\navg. grips:       " + str(np.round(np.average(all_grips), decimals=2)) +
          "\navg. collisions:  " + str(np.round(np.average(all_collisions), decimals=2)) +
          "\navg. target dist: " + str(np.round(np.average(all_target_distances), decimals=2)) +
          "\navg. goal dist:   " + str(np.round(np.average(all_goal_distances), decimals=2)) +
          "\navg. steps:       " + str(np.round(np.average(all_steps), decimals=2)) +
          "\navg. return:      " + str(np.round(np.average(all_returns), decimals=2)))
    env.close()

if MODE == "demo":
    unity_env = UnityEnvironment(f"./{ENV_NAME}", no_graphics=False,
                                 side_channels=[config_channel, log_channel])
    env = UnityToGymWrapper(unity_env, allow_multiple_obs=False)
    config_channel.set_configuration_parameters(time_scale=1.0)
    model = SAC.load(f"models/sac_{ENV_NAME}_{TRAIN_STEPS}", env=env)
    obs = env.reset()
    episodes = 0
    while episodes < DEMO_EPISODES:
        action, _states = model.predict(obs, deterministic=True)
        obs, _reward, done, _info = env.step(action)
        if done:
            episodes += 1
            obs = env.reset()
    env.close()
