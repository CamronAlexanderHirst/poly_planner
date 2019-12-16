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
xBounds = [-200, 600]
yBounds = [-300, 300]
region = Polygon([(xBounds[0], yBounds[0]),
                  (xBounds[1], yBounds[0]),
                  (xBounds[1], yBounds[1]),
                  (xBounds[0], yBounds[1])])
obs_space = []
t = 1

# Polygon Parameters:
nodes = [Point(-100, -200),
         Point(200, -200),
         Point(500, -200),
         Point(500, 200),
         Point(200, 200),
         Point(-100, 200)]

n = 20 # number of samples
TP = 0.90 # Threshold Probability for adding node
vel_mu = [(0,0), (5, math.pi/2), (0,0), (0,0), (5, math.pi/2), (0,0)]
vel_sig = [(0,0), (.5,.2), (0,0), (0,0), (.5,.2), (0,0)]
PPC = PolyProbChecker(nodes, vel_mu, vel_sig, n, TP)

Root = Point(0, 0)
Goal = Point(400, 0)
controls = [math.radians(-30), math.radians(30)]
epsilon = 10 # radius of ball around goal

#Init RRT state sampler
#last arg is goal samp prob
state_sampler = RRT_statesampler(xBounds, yBounds, Goal, 0.1)
#Init RRT control sampler
control_sampler = RRT_controlsampler(controls)
# Init airplane model
# initialize kinematic unicycle w/ constant speed 18m/s
plane = unicycle_model(18, obs_space, region, PPC)
# Init RRT Tree
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
fig = plt.figure()
ax = fig.add_subplot(111)  # create a subplot
ax.axis('equal')

for node in Tree.nodes:
    ax.plot(node.location.x, node.location.y, 'kD', markersize=5)

for i in range(len(graph_path) - 1):
    n1 = graph_path[i]
    n2 = graph_path[i + 1]
    ax.plot([n1.location.x, n2.location.x], [n1.location.y, n2.location.y], 'orange')

ax.plot(Goal.x, Goal.y, 'or', markersize=10)
ax.plot(Root.x, Root.y, 'og', markersize=10)
plt.plot(*region.exterior.xy, 'k')

# pick out time indices of polygons to plot
# find smallest polygon at that time
# plot polygon with node locations!
indices = [0, math.ceil(len(graph_path)/2), len(graph_path)-1]
poly_list = []
for i in indices:
    time = graph_path[i].time
    a_best = math.inf
    p_best = None
    for poly in PPC.poly_list:
        a = poly.poly_hist[time].area
        if a < a_best:
            a_best = a
            p_best = poly.poly_hist[time]
    poly_list.append(p_best)

for poly in poly_list:
    plt.plot(*poly.exterior.xy, 'b')

plt.title('RRT Planner')
plt.xlabel('x (m)')
plt.ylabel('y (m)')

plt.savefig('figures/fig.png', bbox_inches='tight')
plt.show()
