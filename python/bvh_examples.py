from pathfinder.navmesh.navmesh_bvh import NavmeshBVH
from pathfinder.navmesh.navmesh_node import NavmeshNode
import random
import time


def grid_benchmark():
    '''Generate plane grid from suqare polygons and find index of the polygon for random points
    '''
    # generate vertices
    n = 200
    m = 200
    grid_size = 1.0
    vertices = [(grid_size * i, 0.0, grid_size * j) for i in range(n) for j in range(m)]
    polygons = []
    sizes = []
    index = 0
    for i in range(n - 1):
        for j in range(m - 1):
            sizes.append(4)
            polygons.append(m * i + j)
            polygons.append(m * (i + 1) + j)
            polygons.append(m * (i + 1) + j + 1)
            polygons.append(m * i + j + 1)
            index += 1
    # create nodes list
    nodes = []
    index = 0
    for i in range(len(sizes)):
        nodes.append(NavmeshNode(vertices, i, polygons[index:index + sizes[i]]))
        index += sizes[i]

    # create bvh-tree
    start_time = time.time()
    tree = NavmeshBVH(nodes)
    print("generate time:", time.time() - start_time, "seconds")

    # start random sampling
    start_time = time.time()
    samples_count = 10000
    for s in range(samples_count):
        # generate random point
        point = (random.uniform(0.0, (n - 1) * grid_size), 0.0, random.uniform(0.0, (m - 1) * grid_size))
        sample_node = tree.sample(point)
    print("make", samples_count, "samples:", time.time() - start_time, "seconds")

if __name__ == "__main__":
    grid_benchmark()
