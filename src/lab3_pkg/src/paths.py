import numpy as np
import math
from math import sin, cos, asin, acos, atan2, sqrt
from utils import *
from matplotlib import pyplot as plt

class MotionPath:
    def target_state(self, s):
        """
        Target position of turtlebot given the path length s

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        raise NotImplementedError()

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the path length s

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        raise NotImplementedError()

    @property
    def total_length(self):
        """ total path length
        Returns
        -------
        float
            total path length
        """
        raise NotImplementedError()

    @property
    def end_state(self):
        """ Final state after completing the path
        Returns
        -------
        :obj:`numpy.ndarray`
            Final state after completing the path
        """
        return self.target_state(self.total_length)

class ArcPath(MotionPath):
    def __init__(self, radius, angle, left_turn):
        """
        Parameters
        ----------
        radius: float
            how big of a circle in meters
        angle: float
            how much of the circle do you want to complete (in radians).
            Can be positive or negative
        left_turn: bool
            whether the turtlebot should turn left or right
        """
        self.radius = radius
        self.angle = angle
        self.left_turn = left_turn
        self.center = np.array([-self.radius, 0])
        self.target_velocity_norm = 1
        self.time = self.total_length / self.target_velocity_norm

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Circular Arc Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        angle = s / self.total_length / self.angle
        pos = self.center + self.radius * np.array([self.cos(angle), self.sin(angle)])
        if self.left_turn:
            return np.array([pos[0], pos[1], angle])
        else:
            return np.array([-pos[0], pos[1], -angle])

    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Circular Arc Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        angle = s / self.total_length / self.angle
        theta_dot = self.angle / self.time
        pos_dot = self.target_velocity_norm * np.array([self.cos(angle + np.pi/2), self.sin(angle + np.pi/2)])
        if self.left_turn:
            return np.array([pos_dot[0], pos_dot[1], theta_dot])
        else:
            return np.array([-pos_dot[0], pos_dot[1], -theta_dot])

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        return self.angle * self.radius

class LinearPath(MotionPath):
    def __init__(self, length):
        """
        Parameters
        ----------
        length: float
            length of the path
        """
        self.length = length
        self.speed = 1.0

    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Linear Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        return np.array([0, s, 0])


    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Linear Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        # YOUR CODE HERE
        return np.array([0, self.speed, 0])

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return self.length

class ChainPath(MotionPath):
    def __init__(self, subpaths):
        """
        Parameters
        ----------
        subpaths: :obj:`list` of :obj:`MotionPath`
            list of paths which should be chained together
        """
        self.subpaths = subpaths

    def get_subpath_index_and_displacement(self, s):
        path_lengths = [path.total_length() for path in self.subpaths]
        for i in range(len(path_lengths)):
            length = path_lengths[i]
            s -= length
            if s < 0:
                return i, s + length


    def target_state(self, s):
        """
        Target position of turtlebot given the current path length s for Chained Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target position of turtlebot
        """
        i, s = self.get_subpath_index_and_displacement(s)
        return self.subpaths[i].target_state(s)


    def target_velocity(self, s):
        """
        Target velocity of turtlebot given the current path length s for Chained Path

        Parameters
        ----------
        s: float
            the path length the turtlebot should have travelled so far

        Returns
        -------
        :obj:`numpy.ndarray`
            target velocity of turtlebot
        """
        # YOUR CODE HERE
        i, s = self.get_subpath_index_and_displacement(s)
        return self.subpaths[i].target_velocity(s)

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return sum([path.total_length() for path in self.subpaths])

def compute_obstacle_avoid_path(dist, obs_center, obs_radius):
    # YOUR CODE HERE
    raise NotImplementedError()

def plot_path(path):
    """
    Plots on a 2D plane, the top down view of the path passed in

    Parameters
    ----------
    path: :obj:`MotionPath`
        Path to plot
    """
    s = np.linspace(0, path.total_length, 1000, endpoint=False)
    twists = np.array(list(path.target_state(si) for si in s))

    plt.plot(twists[:,0], twists[:,1])
    plt.show()

# YOUR CODE HERE
parallel_parking_path = ChainPath([])

# YOUR CODE HERE
three_point_turn_path = ChainPath([])

if __name__ == '__main__':
    path = three_point_turn_path
    # path = compute_obstacle_avoid_path()
    print(path.end_state)
    plot_path(path)
