from geometry_msgs.msg import Twist
import numpy as np

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
		k1, k2, k3 = 0.3, 0.3, 0.3

		target_s = self.path.target_state(s)
		target_v = self.path.target_velocity(s)

		x, y, theta = current_state
		x_r, y_r, theta_r = target_s
		v_r = (target_v[0]**2 + target_v[1]**2)**0.5 * self.path.sgn(s)
		w_r = target_v[2]

		C = np.matrix([[np.cos(theta_r - theta), 0],[0, 1]])
		ref = np.matrix([v_r, w_r]).T
		u = np.matrix([-k1*(x_r - x), k2*v_r*np.sinc(theta_r - theta)*(y_r - y) - k3*(theta_r - theta)]).T

		desired = np.matmul(C, ref) - u

		angular_vel = desired[1][0][0]
		if angular_vel < -np.pi:
			angular_vel = -np.pi
		elif angular_vel > np.pi:
			angular_vel = np.pi

		vel_msg = Twist()
		vel_msg.linear.x = v_r #desired[0][0][0]
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0 
		vel_msg.angular.y = 0
		vel_msg.angular.z = w_r #desired[1][0][0]
		return vel_msg
