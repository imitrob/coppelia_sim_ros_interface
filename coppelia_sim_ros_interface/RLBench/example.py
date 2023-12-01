from rlbench.environment import Environment
from rlbench.action_modes import ActionMode, ArmActionMode
from rlbench.tasks import FS10_V1
import numpy as np

action_mode = ActionMode(ArmActionMode.ABS_JOINT_VELOCITY)
env = Environment(action_mode)
env.launch()

train_tasks = FS10_V1['train']
test_tasks = FS10_V1['test']
task_to_train = np.random.choice(train_tasks, 1)[0]
task = env.get_task(task_to_train)
task.sample_variation()  # random variation
descriptions, obs = task.reset()
obs, reward, terminate = task.step(np.random.normal(size=env.action_size))
