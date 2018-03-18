import numpy as np
import math
from math import sin, cos, asin, acos, atan2, sqrt
from utils import *
from matplotlib import pyplot as plt
# from main import target_speed as TARGET_SPEED
TARGET_SPEED = 0.2

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
        self.radius = max(radius, 1e-8)
        self.angle = angle
        if np.abs(self.angle) < 1e-8:
            self.angle = 1e-8
        self.left_turn = left_turn
        self.center = np.array([-self.radius, 0])
        self.target_velocity_norm = TARGET_SPEED
        self.time = self.total_length / self.target_velocity_norm
        self.sgn = 1 if self.angle >= 0 else -1

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
        angle = s / self.total_length * self.angle
        pos = self.center + self.radius * np.array([np.cos(angle), np.sin(angle)])
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
        angle = s / self.total_length * self.angle
        theta_dot = self.angle / self.time
        pos_dot = self.target_velocity_norm * self.sgn * np.array([np.cos(angle + np.pi/2), np.sin(angle + np.pi/2)])
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
        return abs(self.angle * self.radius)

class LinearPath(MotionPath):
    def __init__(self, length):
        """
        Parameters
        ----------
        length: float
            length of the path
        """
        self.length = length
        self.speed = TARGET_SPEED
        self.sgn = 1 if self.length >= 0 else -1

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
        return np.array([0, s * self.sgn, 0])


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
        return np.array([0, self.speed * self.sgn, 0])

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return abs(self.length)

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
        path_lengths = [path.total_length for path in self.subpaths]
        for i in range(len(path_lengths)):
            length = path_lengths[i]
            s -= length
            if s < 0:
                return i, s + length
        return len(path_lengths) - 1, self.subpaths[len(path_lengths)-1].total_length

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
        target_state = self.subpaths[i].target_state(s)
        for j in reversed(range(0, i)):
            target_state = rotate_about_endpoint(target_state, self.subpaths[j].end_state)
        target_state[2] = std_range(target_state[2])
        return target_state

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
        target_velocity = self.subpaths[i].target_velocity(s)
        target_velocity[2] = std_range(target_velocity[2])
        return target_velocity

    def sgn(self, s):
        i, s = self.get_subpath_index_and_displacement(s)
        return self.subpaths[i].sgn

    @property
    def total_length(self):
        """ total length of the path
        Returns
        -------
        float
            total length of the path
        """
        # YOUR CODE HERE
        return sum([path.total_length for path in self.subpaths])

def compute_obstacle_avoid_path(dist, obs_center, obs_radius):
    obs_angle = np.arctan2(obs_center[1], obs_center[0]) - np.pi/2
    subpaths = []
    # turn to face obs
    subpaths.append(ArcPath(0, obs_angle, left_turn=True))
    # execute first half
    obs_dist = np.linalg.norm(obs_center)
    subpaths.extend(get_half_obs_path(obs_dist, obs_radius))
    # execute second half
    subpaths.extend(list(reversed(get_half_obs_path(dist - obs_dist, obs_radius))))
    return ChainPath(subpaths)

def get_half_obs_path(obs_y, obs_radius):
    tan_dist = np.sqrt(obs_y**2 - obs_radius**2)

    tan_point = get_tan_point(obs_y, obs_radius, tan_dist)
    tan_angle = np.arctan2(tan_point[1], tan_point[0]) - np.pi/2

    subpaths = []
    subpaths.append(ArcPath(0, tan_angle, left_turn=True))
    subpaths.append(LinearPath(tan_dist))
    subpaths.append(ArcPath(obs_radius, -tan_angle, left_turn=True))
    return subpaths

def get_tan_point(obs_y, obs_radius, tan_dist):
    k = tan_dist
    r = obs_radius
    y = obs_y
    a = np.sqrt(-k**4 + 2*k**2 * (r**2+y**2) - (r**2-y**2)**2) / (2*y)
    b = (k**2 - r**2 + y**2) / (2*y)
    return np.array([a, b])

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
parallel_parking_path = ChainPath([
    LinearPath(0.5),
    ArcPath(0, np.pi/8, left_turn=False),
    LinearPath(-1/np.cos(np.pi/6)),
    ArcPath(0, np.pi/8, left_turn=True),
    LinearPath(0.5)
])

# YOUR CODE HERE
three_point_turn_path = ChainPath([
    ArcPath(1, np.pi/3, left_turn=False),
    ArcPath(1, -np.pi/3, left_turn=True),
    ArcPath(1, np.pi/3, left_turn=False)
])

linear_path = ChainPath([LinearPath(2)])

if __name__ == '__main__':
    # path = three_point_turn_path
    path = compute_obstacle_avoid_path(1, vec(0, 0.5), 0.2)
    print(path.end_state)
    plot_path(path)
