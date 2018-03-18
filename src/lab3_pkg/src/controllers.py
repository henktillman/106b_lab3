from geometry_msgs.msg import Twist
import numpy as np
from utils import *

class Controller():
	def __init__(self, path, k, target_speed, obstacle, obstacle_center, obstacle_radius):
		self.path = path
		self.k = k
		self.target_speed = target_speed
		self.obstacle = obstacle
		self.obstacle_center = obstacle_center
		self.obstacle_radius = obstacle_radius

	def step_path(self, current_state, s):
		"""
		Takes the current state and final state and returns the twist command to reach the target state
		according to the path variable

		Parameters
		----------
		current_state: :obj:`numpy.ndarray`
			twist representing the current state of the turtlebot.  see utils.py
		s: float
			the path length the turtlebot should have travelled so far

		Returns
		-------
		:obj:`geometry_msgs.msg.Twist`
			Twist message to be sent to the turtlebot

		"""
		# Control parameters
		# k1, k2, k3 = 0.3, 0.3, 0.3

		target_s = self.path.target_state(s)
		target_v = self.path.target_velocity(s)

		x, y, theta = current_state
		x_r, y_r, theta_r = target_s
		v_r = (target_v[0]**2 + target_v[1]**2)**0.5 * self.path.sgn(s)
		w_r = target_v[2]

		d_theta = std_range(theta_r - theta)

		dist = np.linalg.norm(current_state[:2] - target_s[:2])
		theta = np.arctan2(target_s[1] - current_state[1], target_s[0] - current_state[0]) - current_state[2]
		dy = dist * np.sin(theta)

		# C = np.array([[np.cos(d_theta), 0],[0, 1]])
		# ref = np.array([v_r, w_r])
		# u = np.array([-k1*(y_r - y), k2*v_r*np.sinc(d_theta)*(x_r - x) - k3*(d_theta)])

		# desired = np.dot(C, ref) - u

		# angular_vel = desired[1]

		vel_msg = Twist()
		vel_msg.linear.x = v_r + 1.5*dy #desired[0]
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0 
		vel_msg.angular.y = 0
		vel_msg.angular.z = w_r + 2.5*d_theta #desired[1]
		return vel_msg
