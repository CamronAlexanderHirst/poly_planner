"""
Create a few objects to make life easier
"""
import numpy as np
import math
from shapely.geometry import Point
from math import radians, cos, sin


class RRT_statesampler:

    def __init__(self, xBounds, yBounds, Goal, pGoal):
        self.xBounds = xBounds
        self.yBounds = yBounds
        self.Goal = Goal
        self.pGoal = pGoal

    def sample(self):
        s = np.random.uniform()
        if s < self.pGoal:
            return self.Goal
        x = np.random.uniform(self.xBounds[0], self.xBounds[1])
        y = np.random.uniform(self.yBounds[0], self.yBounds[1])
        return Point(x, y)


class RRT_controlsampler:

    def __init__(self, controls):
        self.controls = controls

    def sample(self):
        u =  np.random.uniform(self.controls[0], self.controls[-1])
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
                d_opt = d
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
    def __init__(self, state, index, time):
        self.state = state
        self.index = index
        self.location = Point(state[0], state[1])
        self.time = time

    def dist_to(self, point):
        return self.location.distance(point)



class RRT_edge:

    def __init__(self, parent, child, control, path):
        self.parent = parent
        self.child = child
        self.control = control
        self.path = path



class unicycle_model:

    def __init__(self, speed, obs_space, region, PPC):
        self.speed = speed
        self.region = region
        self.obs_space = obs_space
        self.turn_limit = radians(30)
        self.PPC = PPC

    def traj_gen(self, near_node, u_samp, t):
        n = 10
        dt = t/n
        t_0 = near_node.time
        state = near_node.state
        control = u_samp
        edge_path = [state]
        for i in range(n):
            state = self.prop_state(state, control, dt)
            in_region = self.check_in_region(state)
            if in_region:
                edge_path.append(state)
            else:
                return False, None, None, None, None
        x_final = state
        t_final = t_0 + t
        in_poly = self.PPC.check_prob_threshold(x_final, t_final)
        if not in_poly:
            return False, None, None, None, None
        final_point = Point(state[0], state[1])
        return True, x_final, final_point, edge_path, t_final


    def check_in_obs(self, state):
        point = Point(state[0], state[1])
        for obs in self.obs_space:
            if obs.contains(point):
                return True
        return False


    def check_in_region(self, state):
        point = Point(state[0], state[1])
        if self.region.contains(point):
            return True
        return False


    def check_in_poly(self, state):
        point = Point(state[0], state[1])
        i = 0
        for poly in self.poly_list:
            if poly.contains(point):
                i = i + 1

        prob = i/len(self.poly_list)
        if prob > threshold_prob:
            return True
        else:
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
        if theta < 0:
           theta = theta + 2*math.pi


        new_state = [x, y, theta]

        return new_state
