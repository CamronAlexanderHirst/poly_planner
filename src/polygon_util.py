"""
Utility Functions for stoichastic polygons
Assume:
    - polygon velocities are time invariant
"""
from shapely.geometry import MultiPoint, Point


class poly:
    # nodes:  a tuple of Shapely Point objects
    # velocities: a tuple of tuples containing (xvel, yvel) for each node
    def __init__(self, nodes, velocities):
        self.nodes = nodes
        self.velocities = velocities

        # store polygon history for fast lookup
        self.polyhist = {}
        self.polyhist[0] = MultiPoint([self.nodes])


    def prop_poly(self, t):
        # assuming the points of the polygon move w/ vel of wind, prop them.
        nodes_t = ()

        for i in range(len(self.nodes)):
            node = self.nodes[i]
            vel = self.velocities[i]
            x = node.x
            y = node.y
            node_t = Point(x + t*vel[0], x + t*vel[1])
            nodes_t = (Point,) + nodes_t

        self.polyhist[t] = MultiPoint([nodes_t])


    def check_validity(self, point, t):
        if t in self.polyhist:
            poly_t = self.polyhist[t]
        else:
            self.prop_poly(t)
            poly_t = self.polyhist[t]

        if poly_t.contains(point):
            return True
        else:
            return False
