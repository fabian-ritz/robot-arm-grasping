import numpy as np
import torch
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from string_log_channel import StringLogChannel
from tensorboard_callback import TensorboardCallback

MODE = "eval"  # train, eval, demo
ENV_NAME = "lift5d"

TRAIN_STEPS = 5000000
EVAL_EPISODES = 500
DEMO_EPISODES = 50

config_channel = EngineConfigurationChannel()
log_channel = StringLogChannel()

# Custom metrics in tensorboard
logging_callback = TensorboardCallback(log_channel)

# Save a checkpoint every 100000 steps
checkpoint_callback = CheckpointCallback(
  save_freq=100000,
  save_path="./models/checkpoints",
  name_prefix=f"ppo_{ENV_NAME}_{TRAIN_STEPS}",
  save_replay_buffer=True,
  save_vecnormalize=True
)

if MODE == "train":
    unity_env = UnityEnvironment(f"./{ENV_NAME}", no_graphics=True,
                                 side_channels=[config_channel, log_channel])
    env = UnityToGymWrapper(unity_env, allow_multiple_obs=False)
    config_channel.set_configuration_parameters(time_scale=10.0)

    # this would be used to train the 6000000 steps model with gaussian exploration
    # model = PPO(env=env,
    #            policy="MlpPolicy",
    #            policy_kwargs=dict(net_arch=dict(pi=[64, 64, 64], vf=[64, 64, 64]),
    #                               optimizer_kwargs=dict(betas=(0.999, 0.999))),
    #            gae_lambda=0.99,
    #            n_steps=4096,
    #            batch_size=128,
    #            verbose=1,
    #            tensorboard_log=f"./tensorboard/ppo_{ENV_NAME}_{TRAIN_STEPS}")
    # model.learn(TRAIN_STEPS, callback=[logging_callback, checkpoint_callback])
    # model.save(f"models/ppo_{ENV_NAME}_{TRAIN_STEPS}")

    model = PPO(env=env,
                policy='MlpPolicy',
                batch_size=128,
                n_steps=8192,  # authors used 512 samples * 16 envs in their paper
                gamma=0.99,
                gae_lambda=0.9,
                n_epochs=20,
                ent_coef=0.0,
                use_sde=True,
                sde_sample_freq=8,
                max_grad_norm=0.5,
                vf_coef=0.5,
                learning_rate=0.00003,
                clip_range=0.4,
                policy_kwargs=dict(log_std_init=-2,
                                   ortho_init=False,
                                   activation_fn=torch.nn.ReLU,
                                   net_arch=dict(pi=[256, 256], vf=[256, 256]),
                                   optimizer_kwargs=dict(betas=(0.999, 0.999))),
                verbose=1,
                tensorboard_log=f"./tensorboard/ppo_{ENV_NAME}_{TRAIN_STEPS}_sde")
    model.learn(TRAIN_STEPS, callback=[logging_callback, checkpoint_callback])
    model.save(f"models/ppo_{ENV_NAME}_{TRAIN_STEPS}_sde")
    env.close()

if MODE == "eval":
    unity_env = UnityEnvironment(f"./{ENV_NAME}", no_graphics=True,
                                 side_channels=[config_channel, log_channel])
    env = UnityToGymWrapper(unity_env, allow_multiple_obs=False)
    config_channel.set_configuration_parameters(time_scale=10.0)
    model = PPO.load(f"models/ppo_{ENV_NAME}_{TRAIN_STEPS}_sde", env=env)
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
    model = PPO.load(f"models/ppo_{ENV_NAME}_{TRAIN_STEPS}_sde", env=env)
    obs = env.reset()
    episodes = 0
    while episodes < DEMO_EPISODES:
        action, _states = model.predict(obs, deterministic=True)
        obs, _reward, done, _info = env.step(action)
        if done:
            episodes += 1
            obs = env.reset()
    env.close()
