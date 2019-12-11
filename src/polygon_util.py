"""
Utility classes for stoichastic polygons
To make life easier....
Assume:
    - polygon velocities are time invariant
    - polygon is NOT necessarily convex.
"""
from shapely.geometry import MultiPoint, Point, Polygon
import matplotlib.pyplot as plt
import numpy as np
import math


class PolyProbChecker:
    # Polygon Probability Checker
    def __init__(self, nodes, vel_mu, vel_sig, n, TP):
        self.poly_list = []
        self.n = n # number of polygons
        self.threshold_prob = TP

        vel_list = self.gen_vels(vel_mu, vel_sig, n)
        for vel in vel_list:
            p = poly(nodes, vel)
            self.poly_list.append(p)


    def gen_vels(self, vel_mu, vel_sig, n):
        vel_list = []
        for i in range(n):
            list = []
            for j in range(len(vel_mu)):
                speed = np.random.normal(vel_mu[j][0], vel_sig[j][0])
                heading = np.random.normal(vel_mu[j][1], vel_sig[j][1])
                xvel = speed*math.cos(heading)
                yvel = speed*math.sin(heading)
                list.append((xvel, yvel))
            vel_list.append(list)
        return vel_list


    def check_prob(self, point, t):
        # check the probability of Point point being in polygon at time t.
        count = 0
        for poly in self.poly_list:
            if poly.contains_point(point, t):
                count = count + 1
        n = len(self.poly_list)
        return count/n

    def check_prob_threshold(self, state, t):
        point = Point(state[0], state[1])
        prob = self.check_prob(point,t)
        if prob >= self.threshold_prob:
            return True
        else:
            return False



class poly:
    # nodes:  a list of Shapely Point objects
    # velocities: a list of tuples containing (xvel, yvel) for each node
    def __init__(self, nodes, velocities):
        self.nodes = nodes
        self.velocities = velocities

        # store polygon history for fast lookup
        self.poly_hist = {}
        self.poly_node_hist = {}
        self.poly_node_hist[0] = self.nodes
        self.poly_hist[0] = Polygon([p.coords[:][0] for p in self.nodes])


    def prop_poly(self, t):
        # assuming the points of the polygon move w/ vel of wind, prop them.
        nodes_t = []

        for i in range(len(self.nodes)):
            node = self.nodes[i]
            vel = self.velocities[i]
            x = node.x
            y = node.y
            node_t = Point(x + t*vel[0], y + t*vel[1])
            nodes_t.append(node_t)

        self.poly_node_hist[t] = nodes_t
        self.poly_hist[t] = Polygon([p.coords[:][0] for p in nodes_t])


    def contains_point(self, point, t):
        if t in self.poly_hist:
            poly_t = self.poly_hist[t]
        else:
            self.prop_poly(t)
            poly_t = self.poly_hist[t]

        if poly_t.contains(point):
            return True
        else:
            return False


if __name__ == "__main__":

    nodes = [Point(0,0), Point(100,0), Point(100,100), Point(0, 100)]
    n = 10
    vel_mu = [(0,0), (0,0), (0,0), (0,0)]
    vel_sig = [(1, math.pi), (0,0), (0,0), (0,0)]

    PPC = PolyProbChecker(nodes, vel_mu, vel_sig, n, 0.95)
    prob = PPC.check_prob(Point(2,2), 5)
    print(prob)


###############################################################################
# Plotting
###############################################################################

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)  # create a subplot
    ax.axis('equal')

    for PCCpoly in PPC.poly_list:
        polygon = PCCpoly.poly_hist[5]
        ax.plot(*polygon.exterior.xy)

    plt.show()
