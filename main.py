"""
Planning in Polygons!

Author: Alex Hirst
"""

from src.RRT_util import *
from src.polygon_util import *
from shapely.geometry import Point, Polygon
import math
import time
import matplotlib.pyplot as plt

# Environment Parameters (m)
xBounds = [-100, 500]
yBounds = [-200, 200]
region = Polygon([(xBounds[0], yBounds[0]), (xBounds[1], yBounds[0]),
                  (xBounds[1], yBounds[1]), (xBounds[0], yBounds[1])])
obs_space = []
t = 1

# Polygon Parameters:
nodes = [Point(-100,-200), Point(500,-200), Point(500,200), Point(-100, 200)]
n = 100 # number of samples
TP = 0.95 # Threshold Probability for adding node
vel_mu = [(0,0), (0,0), (0,0), (0,0)]
vel_sig = [(8, math.pi), (0, math.pi), (0, math.pi), (8, math.pi)]
PPC = PolyProbChecker(nodes, vel_mu, vel_sig, n, TP)

Root = Point(0, 0)
Goal = Point(400, 0)
controls = [math.radians(-30), math.radians(30)]
epsilon = 25 # radius of ball around goal

#Init RRT state sampler
#last arg is goal samp prob
state_sampler = RRT_statesampler(xBounds, yBounds, Goal, 0.05)
#Init RRT control sampler
control_sampler = RRT_controlsampler(controls)
# Init airplane model
# initialize kinematic unicycle w/ constant speed 18m/s
plane = unicycle_model(18, obs_space, region, PPC)
# Init RRT Tree
root_state = [Root.x, Root.y, 0, 0]
Root_node = RRT_node(root_state, 0, 0)
Tree = RRT_tree(Root_node)

j = 0
while j < 11:
    j = j + 1

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
    # add node trajectory, if it exists
    if x_new != None:
        new_node = RRT_node(x_new, index, t_f_new)
        new_edge = RRT_edge(near_node.index, new_node.index, u_new, path_new)
        Tree.nodes.append(new_node)
        Tree.edges.append(new_edge)
        Tree.parents[new_node.index] = near_node.index
    # if a path is found, break and return the path
    print("number of nodes", len(Tree.nodes))
    if new_node.dist_to(Goal) < epsilon:
        graph_path = Tree.path_from_start(new_node.index)
        print(graph_path)
        break
end_time = time.time()
print("Runtime (s): ",end_time - start_time)


###############################################################################
# Plotting
###############################################################################

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111)  # create a subplot
ax.axis('equal')

for node in Tree.nodes:
    ax.plot(node.location.x, node.location.y, 'D', markersize=5)

ax.plot(Goal.x, Goal.y, 'or', markersize=10)
ax.plot(Root.x, Root.y, 'og', markersize=10)

plt.show()
