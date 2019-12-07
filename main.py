"""
Planning in Polygons!

Author: Alex Hirst

"""

from src.RRT_util import *
from src.polygon_util import *
from shapely.geometry import Point
import math

# Environment Parameters (m)
xBounds = [-500, 500]
yBounds = [-500, 500]
obs_space = []
t = 1


Root = Point(100, 0)
Goal = Point(400, 0)

#controls = [math.radians(-30), math.radians(-15), 0, math.radians(15), math.radians(30)]
controls = [0]
epsilon = 25 # radius of ball around goal

#Init RRT state sampler
state_sampler = RRT_statesampler(xBounds, yBounds)

#Init RRT control sampler
control_sampler = RRT_controlsampler(controls)

# Init airplane model
plane = unicycle_model(18, obs_space) # initialize kinematic unicycle w/ constant speed 18m/s

# Init RRT Tree
root_state = [Root.x, Root.y, 0, 0]
Root_node = RRT_node(root_state, 0)
Tree = RRT_tree(Root_node)

j = 0
while j < 10:
    j = j + 1
while True:
    print('new_loop')
    # Sample Random State
    x_samp = state_sampler.sample()
    #print(x_samp)

    # find state nearest to sample state
    near_node = Tree.find_nearest_node(x_samp)

    print(near_node.index)
    print(near_node.state)

    # sample a bunch of control inputs for dt, find the one which achieves closest
    d_opt = math.inf
    x_new = None
    u_new = None
    for i in range(20):
        u_samp = control_sampler.sample()
        traj_valid, x_final, final_point, edge_path = plane.traj_gen(near_node, u_samp, t)

        if traj_valid:
            d = near_node.dist_to(final_point)
            if d < d_opt:
                x_new = x_final
                u_new = u_samp
                d_opt = d
                index = len(Tree.nodes)

    # add node trajectory, if it exists
    print(x_final)
    if x_new != None:
        new_node = RRT_node(x_new, index)
        new_edge = RRT_edge(near_node.index, new_node.index, u_new)
        Tree.nodes.append(new_node)
        Tree.edges.append(new_edge)
        Tree.parents[new_node.index] = near_node.index

    # if a path is found, break and return the path
    print(new_node.state)
    print(new_node.dist_to(Goal))
    if new_node.dist_to(Goal) < epsilon:
        graph_path = Tree.path_from_start(new_node.index)
        break


###############################################################################
# Plotting
###############################################################################
