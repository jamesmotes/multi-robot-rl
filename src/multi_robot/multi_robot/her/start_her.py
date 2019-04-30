#!/usr/bin/env python
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time
import liveplot
import qlearn
from her import learn

from multi_robot import multi_robot_env
#import multi_robot.robot_envs
#import multi_robot_env
#from gym.envs.registration import register

#register(
#    id='MultiRobot-v0',
#    entry_point='multi_robot:MultiRobotEnv',
    # More arguments here
#)

def render():
    render_skip = 0 #Skip first X episodes.
    render_interval = 50 #Show render Every Y episodes.
    render_episodes = 10 #Show Z episodes every rendering.

    if (x%render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x-render_episodes)%render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)

if __name__ == '__main__':

    env = gym.make('HERMultiRobot-v0')


    outdir = '~/multi_robot-rl/src/second_output'
    env = gym.wrappers.Monitor(env, outdir, force=True)

    plotter = liveplot.LivePlot(outdir)

    last_time_steps = numpy.ndarray(0)

    epsilon_discount = 0.999 # 1098 eps to reach 0.1

    start_time = time.time()
    total_episodes = 10000
    highest_reward = 0

    learn(env=env)



    env.close()
