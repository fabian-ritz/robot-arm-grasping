from stable_baselines3.common.callbacks import BaseCallback
import string_log_channel


class TensorboardCallback(BaseCallback):

    def __init__(self, log_channel: string_log_channel, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)
        self.log_channel = log_channel

    def _on_step(self) -> bool:
        if self.log_channel.lastMessage is not None:
            episode_results = self.log_channel.lastMessage.replace(",", ".").split(";")
            self.logger.record("custom/ep_success", int(episode_results[0]))
            self.logger.record("custom/ep_target_oob", int(episode_results[1]))
            self.logger.record("custom/ep_score", float(episode_results[2]))
            self.logger.record("custom/ep_grasps", int(episode_results[3]))
            self.logger.record("custom/ep_collisions", int(episode_results[4]))
            self.logger.record("custom/ep_rem_target_dist", float(episode_results[5]))
            self.logger.record("custom/ep_rem_goal_dist", float(episode_results[6]))
            self.log_channel.lastMessage = None
        return True
