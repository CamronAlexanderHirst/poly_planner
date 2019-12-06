"""
Planning in Polygons!

Author: Alex Hirst

"""

from src.RRT_util import *
from src.polygon_util import *
from Shapely import Point

# Environment Parameters (m)
xBounds = [-500, 500]
yBounds = [500, 500]

Root = Point(100, 0)
Goal = Point(400, 0)


# Init RRT Tree
Tree = RRT_tree(Root)

while True:
    # Sample Random State
