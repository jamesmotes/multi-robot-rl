import gym
import rospy
import roslaunch
import time
import numpy as np
import math

from gym import utils, spaces
import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from gym.utils import seeding

#import logging
#from gym.envs.registration import register

#logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# Turtlebot envs
#register(
#    id='MyMultiRobot-v0',
#    entry_point='multi_robot.scripts.multi_robot_env:MultiRobotEnv',
    # More arguments here
#)

class HERMultiRobotEnv(gazebo_env.GazeboEnv,utils.EzPickle):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "multi_robot.launch")
        self.rob1_vel_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=5)
        self.rob2_vel_pub = rospy.Publisher('/robot2/mobile_base/commands/velocity', Twist, queue_size=5)

        self.publishers = [self.rob1_vel_pub, self.rob2_vel_pub]
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.get_model_srv = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

        #self.action_space = spaces.Discrete(9) #1,2 x F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()
        self.steps = 0
        self.max_steps = 1000

        utils.EzPickle.__init__(self)

        self.n_actions = 2
        self.n_robots = 2
        self._env_setup()
        obs = self._get_obs()

        #self.action_space = spaces.Box(-1., 1., shape=(self.n_actions,), dtype='float32')
        self.action_space = spaces.Box(-.3, .3, shape=(4,), dtype='float32')
        self.observation_space = spaces.Dict(dict(
            desired_goal=spaces.Box(-np.inf, np.inf, shape=obs['desired_goal'].shape, dtype='float32'),
            achieved_goal=spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype='float32'),
            observation=spaces.Box(-np.inf, np.inf, shape=obs['observation'].shape, dtype='float32'),
        ))

    def discretize_observation(self,data,new_ranges):
        # print("DATA")
        # print(data)
        discretized_ranges = []
        min_range = 0.2
        done = False
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf'):
                    discretized_ranges.append(6)
                elif np.isnan(data.ranges[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(data.ranges[i]))
            if (min_range > data.ranges[i] > 0):
                done = True
        return discretized_ranges,done


    def reward(self,data1,data2):
        #print("DATA 1")
        #print(data1.pose)
        #print("DATA 2")
        #print(data2.pose)
        reward = 0

        done = False
        x_diff = data1.pose.position.x - data2.pose.position.x
        y_diff = data1.pose.position.y - data2.pose.position.y
        z_diff = data1.pose.position.z - data2.pose.position.z
        distance = math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
        if distance < .2:
            done = True #crashed into each other
            reward = -10
            # print("TOO CLOSE")
        elif distance > 5:
            done = True #too far apart
            reward = -5
            # print("TOO FAR APART")
        elif distance < 1:
            reward = 10/distance
        else:
            reward = -1*distance

        if self.steps >= self.max_steps:
            done = True
            # print("REACHED MAX STEPS")
            # print(self.steps)
            self.steps = 0
        return reward,done

    def configure_state(self, data1, data2):
        state = []
        state.append(data1.pose.position.x)
        state.append(data1.pose.position.y)
        state.append(data1.pose.position.z)
        state.append(data1.pose.orientation.x)
        state.append(data1.pose.orientation.y)
        state.append(data1.pose.orientation.z)
        state.append(data1.pose.orientation.w)

        state.append(data2.pose.position.x)
        state.append(data2.pose.position.y)
        state.append(data2.pose.position.z)
        state.append(data2.pose.orientation.x)
        state.append(data2.pose.orientation.y)
        state.append(data2.pose.orientation.z)
        state.append(data2.pose.orientation.w)
        
        return state

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def move_robot(self, robot, action):
        vel_cmd = Twist()
        # if action == 0: #STATIONARY
        #     vel_cmd.linear.x = 0.0
        #     vel_cmd.angular.z = 0.0
        # elif action == 1: #FORWARD 
        #     vel_cmd = Twist()
        #     vel_cmd.linear.x = 0.3
        #     vel_cmd.angular.z = 0.0
        # elif action == 2: #LEFT 
        #     vel_cmd = Twist()
        #     vel_cmd.linear.x = 0.1
        #     vel_cmd.angular.z = 0.3
        # elif action == 3: #RIGHT
        #     vel_cmd = Twist()
        #     vel_cmd.linear.x = 0.1
        #     vel_cmd.angular.z = -0.3
        vel_cmd.linear.x = action[robot-1][0]
        vel_cmd.angular.z = action[robot-1][1]
        self.publishers[robot-1].publish(vel_cmd)
        #print("moving robot")
        #print(robot-1)

    def _set_action(self, action):
        assert action.shape == (2,2)
        action = action.copy()
        self.step(action)

    def step(self, action):

        # print("ACTION")
        action = action.reshape((2,2))
        # print(action)

        self.steps += 1

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

            
        # if action == 0: #1-FORWARD 2-FORWARD
        #     self.move_robot(1,1)
        #     self.move_robot(2,1)
            
        # elif action == 1: #1-LEFT 2-FORWARD
        #     self.move_robot(1,2)
        #     self.move_robot(2,1)
           
        # elif action == 2: #1-RIGHT 2-FORWARD
        #     self.move_robot(1,3)
        #     self.move_robot(2,1)
            
        # elif action == 3: #1-FORWARD 2-LEFT
        #     self.move_robot(1,1)
        #     self.move_robot(2,2)
            
        # elif action == 4: #1-LEFT 2-LEFT
        #     self.move_robot(1,2)
        #     self.move_robot(2,2)
           
        # elif action == 5: #1-RIGHT 2-LEFT
        #     self.move_robot(1,3)
        #     self.move_robot(2,2)
            
        # elif action == 6: #1-FORWARD 2-RIGHT
        #     self.move_robot(1,1)
        #     self.move_robot(2,3)
            
        # elif action == 7: #1-LEFT 2-RIGHT
        #     self.move_robot(1,2)
        #     self.move_robot(2,3)
           
        # elif action == 8: #1-RIGHT 2-RIGHT
        #     self.move_robot(1,3)
        #     self.move_robot(2,3)

        #later turn this into a for loop for self.n_robots
        self.move_robot(1,action)
        self.move_robot(2,action)


        model1 = GetModelStateRequest()
        model1.model_name = 'Robot1'

        results1 = None
        while results1 is None:
            try:
                results1 = self.get_model_srv(model1)
                #print("RESULTS 1")
                #print(results1.pose)
            except:
                pass

        model2 = GetModelStateRequest()
        model2.model_name = 'Robot2'

        results2 = None
        while results2 is None:
            try:
                results2 = self.get_model_srv(model2)
                #print("RESULTS 2")
                #print(results2.pose)
            except:
                pass


        #data = None
        #while data is None:
        #    try:
        #        data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
        #    except:
        #        pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            # print ("/gazebo/pause_physics service call failed")

        #state,done = self.discretize_observation(results1,results2,5)
        reward,done = self.reward(results1,results2)
        state = self.configure_state(results1,results2)
        #if not done:
            #if action == 0:
            #    reward = 5
            #else:
            #    reward = 1
        #else:
        #    reward = -200
        obs = self._get_obs()
        done = self._is_done(obs)
        reward = self._compute_reward(obs,done)

        return obs, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(data,5)

        return self._get_obs()

    def _env_setup(self):#, initial_qpos):
        self.goal = self._sample_goal()
        self.reset()
        self._get_obs()

    def _get_obs(self):

        model1 = GetModelStateRequest()
        model1.model_name = 'Robot1'

        results1 = None
        while results1 is None:
            try:
                results1 = self.get_model_srv(model1)
                #print("RESULTS 1")
                #print(results1.pose)
            except:
                pass

        model2 = GetModelStateRequest()
        model2.model_name = 'Robot2'

        results2 = None
        while results2 is None:
            try:
                results2 = self.get_model_srv(model2)
                #print("RESULTS 2")
                #print(results2.pose)
            except:
                pass

        state = self.configure_state(results1, results2)
        obs = np.array(state)


        achieved_goal = self._sample_achieved_goal(state)
        dictionary = {
            'observation': achieved_goal.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy(),
        }
        # print(dictionary)
        return dictionary

    def _sample_goal(self):
        relative_positions = np.array([.25, 0., .25, 0.], np.float32)
        return relative_positions

    def _sample_achieved_goal(self, state):
        x_diff = (state[0] - state[7])
        y_diff = (state[1] - state[8])
        z_diff = (state[2] - state[9])

        relative_distance = math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)

        angle1 = -1 * math.tan((-1*y_diff)/(-1*x_diff))
        angle1 = angle1 + math.tan(state[1]/state[0])

        angle2 = -1 * math.tan(y_diff/x_diff)
        angle2 = angle2 * math.tan(state[8]/state[9])

        relative_positions = np.array([relative_distance, angle1, relative_distance, angle2], np.float32)
        return relative_positions

    def _is_done(self, observations):
        d = self.goal_distance(observations['achieved_goal'], self.goal)
        #d = d[0]# + d[1] 
        return (d <= 1).astype(np.float32)

    def _compute_reward(self, observations, done):

        d = self.goal_distance(observations['achieved_goal'], self.goal)
        #if self.reward_type == 'sparse':
        #    return -(d > self.distance_threshold).astype(np.float32)
        #else:
        #d = d[0]# + d[1]
        return (-1 * d)
    def compute_reward(self, achieved_goal, desired_goal, info):
        d = self.goal_distance(achieved_goal, desired_goal)
        #d = d[0]# + d[1]
        return (-1 * d)

    def goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _set_init_pose(self):
        """Sets the Robots in their init pose
        """
        self.reset()

        return True
        
    def _init_env_variables(self):
        """
        Inits variables needed to be initialized each time we reset at the start
        of an episode.
        :return:
        """
        pass

















