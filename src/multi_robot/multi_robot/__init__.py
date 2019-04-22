from gym.envs.registration import register

#logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# Turtlebot envs
register(
    id='MyMultiRobot-v0',
    entry_point='multi_robot.scripts.multi_robot_env:MultiRobotEnv',
    # More arguments here
)
register(
    id='HERMultiRobot-v0',
    entry_point='multi_robot.scripts.her_multi_robot_env:HERMultiRobotEnv',
    # More arguments here
)
