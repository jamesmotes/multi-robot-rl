#! /usr/bin/env python

import rospy
from nav.msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('/my_odom', Odometry)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = '/odom'

model = GetModelStateRequest()
model.model_name = 'mobile_base'