# imports may be necessary

class Controller():
	def __init__(self, path, target_speed, obstacle, obstacle_center, obstacle_radius):
		self.path = path
		self.target_speed = target_speed
		self.obstacle = obstacle
		self.obstacle_center = obstacle_center
		self.obstacle_radius = self.obstacle_radius

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
		# YOUR CODE HERE