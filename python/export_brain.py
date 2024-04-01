import torch as th
from stable_baselines3 import PPO, SAC


class PPOBrain(th.nn.Module):
  def __init__(self, policy):
    super().__init__()
    self.policy = policy
    version_number = th.Tensor([3])  # version_number
    self.version_number = th.nn.Parameter(version_number, requires_grad=False)

    memory_size = th.Tensor([0])  # memory_size
    self.memory_size = th.nn.Parameter(memory_size, requires_grad=False)

    num_actions = self.policy.action_space.shape[0]
    output_shape = th.Tensor([num_actions])  # continuous_action_output_shape
    self.output_shape = th.nn.Parameter(output_shape, requires_grad=False)

  def forward(self, observation):
    actions = self.policy(observation, deterministic=True)[0]
    return actions, self.output_shape, self.version_number, self.memory_size


class SACBrain(th.nn.Module):
  def __init__(self, policy):
    super().__init__()
    self.policy = policy
    print(policy)
    version_number = th.Tensor([3])  # version_number
    self.version_number = th.nn.Parameter(version_number, requires_grad=False)

    memory_size = th.Tensor([0])  # memory_size
    self.memory_size = th.nn.Parameter(memory_size, requires_grad=False)

    num_actions = self.policy.action_space.shape
    output_shape = th.Tensor([num_actions])  # continuous_action_output_shape
    self.output_shape = th.nn.Parameter(output_shape, requires_grad=False)

  def forward(self, observation):
    actions = self.policy(observation, deterministic=False)
    return actions, self.output_shape, self.version_number, self.memory_size


MODEL_NAME = 'ppo_lift5d_5000000_sde'
model = PPO.load(f"models/{MODEL_NAME}")
brain = PPOBrain(model.policy)
#MODEL_NAME = 'sac_lift5d_4500000'
#model = SAC.load(f"models/{MODEL_NAME}")
#brain = SACBrain(model.policy.actor)

th.onnx.export(
  brain,
  th.randn(1, *model.observation_space.shape),
  f"models/{MODEL_NAME}.onnx",
  opset_version=9,
  input_names=["obs_0"],  # this name is required for unity ml_agents, do not change!
  output_names=['continuous_actions', 'continuous_action_output_shape', 'version_number', 'memory_size'],
  dynamic_axes={'obs_0': {0: 'batch'},
                'continuous_actions': {0: 'batch'},
                'continuous_action_output_shape': {0: 'batch'}},
  export_params=True,
  do_constant_folding=True,
  verbose=True
)
