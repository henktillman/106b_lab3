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
        # YOUR CODE HERE
        raise NotImplementedError()

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
        # YOUR CODE HERE
        raise NotImplementedError()

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        raise NotImplementedError()

class LinearPath(MotionPath):
    def __init__(self, length):
        """
        Parameters
        ----------
        length: float
            length of the path
        """
        self.length = length

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
        return np.array([0, s])
        

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
        return np.array([0, 1.0])

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
        # YOUR CODE HERE
        raise NotImplementedError()

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
        raise NotImplementedError()

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        raise NotImplementedError()

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
