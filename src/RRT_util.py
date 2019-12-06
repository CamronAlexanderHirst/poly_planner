"""
Create a few objects to make life easier
"""
import numpy as np
from shapely.geometry import Point
from math import radians, cos, sin


class RRT_statesampler
    def __init__(self, xBounds, yBounds):



class RRT_tree:
    def __init__(self, root):
        self.nodes = []
        self.edges = []

        self.nodes.append(root)

class RRT_node:

    def __init__(self, state, control):
        self.state = state
        self.location = Point(state.x, state.y)
        self.control = control

    def dist_to(point):
        return self.location.distance(point)


class RRT_edge:

    def __init__(self, parent, child):
        self.parent = parent
        self.child = child


class unicycle:

    def __init__(self, state, speed):
        self.speed = speed
        self.state = state
        self.control_space = [radians(-5), radians(5)]

    def sample_control(self):
        return np.random.uniform(self.control_space[0], self.control_space[1])

    def prop_state(self, contol, dt):
        # first order Euler integration. At each step, check for:
        # 1) Number of Polygons the state is inside.
        v = self.speed

        x0 = self.state[0]
        y0 = self.state[1]
        theta0 = self.state[2]
        omega0 = self.state[3]

        x = x0 + dt*v*cos(theta0)
        y = y0 + dt*v*sin(theta0)
        theta = theta0 + dt*omega0
        omega = omega0 + dt*control

        new_state = [x, y, theta, omega]

        return new_state
