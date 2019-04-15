from setuptools import setup

setup(name='multi_robot',
      version='0.1',
      description='Multi Robot Reinforcement Learning',
      url='https://github.com/g-and-j/gym-gazebo',
      author='James Motes',
      author_email='jamesmotes@gmail.com',
      license='MIT',
      packages=['multi_robot'],
      install_requires=[
          'markdown',
          'gym',
          'matplotlib',
#          'rospy',
#          'roslaunch',
#          'time',
          'numpy',
#          'gym_gazebo',
#          'geometry_msgs',
#          'std_srvs',
#          'sensor_msgs',
#          'random'
      ],
      zip_safe=False)
