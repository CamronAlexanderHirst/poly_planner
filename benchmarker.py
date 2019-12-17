"""
Benchmarker for Planning in Polygons

Author: Alex Hirst
"""

from src.RRT_util import *
from src.polygon_util import *
from shapely.geometry import Point, Polygon
import math
import pandas as pd
import time
import matplotlib.pyplot as plt

# Environment Parameters (m)
xBounds = [-200, 600]
yBounds = [-300, 300]
region = Polygon([(xBounds[0], yBounds[0]),
                  (xBounds[1], yBounds[0]),
                  (xBounds[1], yBounds[1]),
                  (xBounds[0], yBounds[1])])
obs_space = [] # can add obstacles to environment of desired.

# Polygon Parameters:
nodes = [Point(-100, -200),
         Point(200, -200),
         Point(500, -200),
         Point(500, 200),
         Point(200, 200),
         Point(-100, 200)]
n = 50 # number of samples
TP = 0.90 # Threshold Probability for adding node
vel_mu = [(0,0), (3, math.pi/2), (0,0), (0,0), (3, math.pi/2), (0,0)]
vel_sig = [(0,0), (0.5,.1), (0,0), (0,0), (0.5,.1), (0,0)]

# Aircraft Parameters
t = 1 # aircraft time step
Root = Point(0, 0)
Goal = Point(400, 0)
controls = [math.radians(-30), math.radians(30)]
epsilon = 10 # radius of ball around goal


# number of benchmarking runs:
num_runs = 100
# Create pandas dataframe
rows_list = []
#column_names = ['runtime', 'path length', 'number of nodes']
#df = pd.DataFrame(column_names)

for run in range(num_runs):
    PPC = PolyProbChecker(nodes, vel_mu, vel_sig, n, TP)
    state_sampler = RRT_statesampler(xBounds, yBounds, Goal, 0.1)
    control_sampler = RRT_controlsampler(controls)
    plane = unicycle_model(18, obs_space, region, PPC)
    root_state = [Root.x, Root.y, 0, 0]
    Root_node = RRT_node(root_state, 0, 0)
    Tree = RRT_tree(Root_node)
    start_time = time.time()
    # Main Planning Loop:
    while True:
        # Sample Random State
        x_samp = state_sampler.sample()
        # find state nearest to sample state
        near_node = Tree.find_nearest_node(x_samp)
        # sample a bunch of control inputs for dt, find the one which achieves closest
        d_opt = math.inf
        x_new = None
        u_new = None
        for i in range(10):
            u_samp = control_sampler.sample()
            traj_valid, x_f, point_f, path, t_f = plane.traj_gen(near_node, u_samp, t)
            if traj_valid:
                d = near_node.dist_to(point_f)
                if d < d_opt:
                    x_new = x_f
                    u_new = u_samp
                    t_f_new = t_f
                    path_new = path
                    d_opt = d
                    index = len(Tree.nodes)
        if x_new != None:
            new_node = RRT_node(x_new, index, t_f_new)
            new_edge = RRT_edge(near_node.index, new_node.index, u_new, path_new)
            Tree.nodes.append(new_node)
            Tree.edges.append(new_edge)
            Tree.parents[new_node.index] = near_node.index
        if new_node.dist_to(Goal) < epsilon:
            graph_path = Tree.path_from_start(new_node.index)
            break
        end_time = time.time()
        if end_time - start_time > 300:
            break

    end_time = time.time()
    runtime = end_time - start_time
    number_of_nodes = len(Tree.nodes)

    if runtime < 300:
        path_length = 18 * (len(graph_path) - 1)

        print("Run Number: ", run)
        print("Runtime (s): ",runtime)

        # save the benchmarking data
        dict = {'runtime': runtime, 'pathlength':path_length,
                'numbernodes': number_of_nodes}
        rows_list.append(dict)
    else:
        print('No data collected - runtime too long')
        dict = {'runtime': runtime, 'pathlength':None,
                'numbernodes': None}
        rows_list.append(dict)

df = pd.DataFrame(rows_list)
df.to_pickle("benchmarking_results/benchmark_1.pkl")
