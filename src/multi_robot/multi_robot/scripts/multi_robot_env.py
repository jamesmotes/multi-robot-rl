import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

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

class MultiRobotEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "multi_robot.launch")
        self.rob1_vel_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=5)
        self.rob2_vel_pub = rospy.Publisher('/robot2/mobile_base/commands/velocity', Twist, queue_size=5)
        self.publishers = [rob1_vel_pub, rob2_vel_pub]
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(9) #1,2 x F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def discretize_observation(self,data,new_ranges):
        print("DATA")
        print(data)
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

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def move_robot(self, robot, action):
        vel_cmd = Twist()
        robot = robot - 1
        if action == 0: #STATIONARY
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
        elif action == 1: #FORWARD 
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.3
            vel_cmd.angular.z = 0.0
        elif action == 2: #LEFT 
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = 0.3
        elif action == 3: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = -0.3
        self.publishers[robot].publish(vel_cmd)

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

            
        elif action == 0: #1-FORWARD 2-FORWARD
            self.move_robot(1,1)
            self.move_robot(2,1)
            
        elif action == 1: #1-LEFT 2-FORWARD
            self.move_robot(1,2)
            self.move_robot(2,1)
           
        elif action == 2: #1-RIGHT 2-FORWARD
            self.move_robot(1,3)
            self.move_robot(2,1)
            
        elif action == 3: #1-FORWARD 2-LEFT
            self.move_robot(1,1)
            self.move_robot(2,2)
            
        elif action == 4: #1-LEFT 2-LEFT
            self.move_robot(1,2)
            self.move_robot(2,2)
           
        elif action == 5: #1-RIGHT 2-LEFT
            self.move_robot(1,3)
            self.move_robot(2,2)
            
        elif action == 6: #1-FORWARD 2-RIGHT
            self.move_robot(1,1)
            self.move_robot(2,3)
            
        elif action == 7: #1-LEFT 2-RIGHT
            self.move_robot(1,2)
            self.move_robot(2,3)
           
        elif action == 8: #1-RIGHT 2-RIGHT
            self.move_robot(1,3)
            self.move_robot(2,3)
            

        try:
            odom = rospy.wait_for_message('/robot1/odom', Odom, timeout=5)
            print("ODOM")
            print(odom)
        except:
            print("DIDN'T FIND ANYTHING FOR ODOM")
            pass

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

        state,done = self.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:
            reward = -200

        return state, reward, done, {}

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

        return state
