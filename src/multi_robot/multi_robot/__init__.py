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
