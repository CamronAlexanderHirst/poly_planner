"""
Create a few objects to make life easier
"""
import numpy as np
import math
from shapely.geometry import Point
from math import radians, cos, sin


class RRT_statesampler:

    def __init__(self, xBounds, yBounds):
        self.xBounds = xBounds
        self.yBounds = yBounds

    def sample(self):
        x = np.random.uniform(self.xBounds[0], self.xBounds[1])
        y = np.random.uniform(self.yBounds[0], self.yBounds[1])

        return Point(x, y)


class RRT_controlsampler:

    def __init__(self, controls):
        self.controls = controls

    def sample(self):
        u = np.random.choice(self.controls)
        return u


class RRT_tree:

    def __init__(self, root):
        self.nodes = []
        self.nodes.append(root)
        self.edges = []
        self.parents = {} # key = child, value = parent

    def find_nearest_node(self, point):
        d_opt = math.inf
        node_opt = None
        i_opt = None
        i = 0
        for node in self.nodes:
            d = node.dist_to(point)
            if d < d_opt:
                d = d_opt
                node_opt = node
        return node_opt

    def path_from_start(self, node_index):
        i = node_index
        path = []
        while True:
            parent_index = self.parents[i]
            path.append(self.nodes[parent_index])
            i = parent_index
            if i == 0:
                break
        return path



class RRT_node:

    def __init__(self, state, index):
        self.state = state
        self.index = index
        self.location = Point(state[0], state[1])

    def dist_to(self, point):
        return self.location.distance(point)


class RRT_edge:

    def __init__(self, parent, child, control):
        self.parent = parent
        self.child = child
        self.control = control


class unicycle_model:

    def __init__(self, speed, obs_space):
        self.speed = speed
        self.obs_space = obs_space
        self.turn_limit = radians(30)

    def traj_gen(self, near_node, u_samp, t):
        n = 10
        dt = t/n
        state = near_node.state
        #print("state in traj_gen:", state)
        control = u_samp
        edge_path = [state]
        for i in range(n):
            #print("state in traj:", state)
            state = self.prop_state(state, control, dt)
            in_obs = self.check_in_obs(state)
            if not in_obs:
                edge_path.append(state)
            else:
                return False, None, None, None
        x_final = state
        final_point = Point(state[0], state[1])
        return True, x_final, final_point, edge_path

    def check_in_obs(self, state):
        point = Point(state[0], state[1])
        for obs in self.obs_space:
            if obs.contains(point):
                return True
        return False

    def prop_state(self, state, control, dt):
        # first order Euler integration.
        v = self.speed
        x0 = state[0]
        y0 = state[1]
        theta0 = state[2]
        x = x0 + dt*v*cos(theta0)
        y = y0 + dt*v*sin(theta0)
        theta = theta0 + dt*control


        new_state = [x, y, theta]

        return new_state
