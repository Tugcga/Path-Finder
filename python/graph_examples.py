from pathfinder.navmesh.navmesh_graph import NavmeshGraph


def simple_example():
    '''each graph defined by the following:
        -) array of vertex names
            names are integers, by not necesary from 0 to n-1
        -) vertex positions in R^3
            each position is a 3-tuple (x, y, z)
        -) edges
            each edge is a 2-tuple (a, b), where a and b are vertex names

        this example demonstrate how to build simple graph and find the shortest path between tow vertices
    '''
    graph_vertices = [2, 3, 5, 7, 12, 18]
    graph_positions = [(-1.0, 0.0, 2.0), (-1.0, 0.0, -2.0), (4.0, 0.0, 0.0), (2.0, 0.0, 2.0), (-2.0, 0.0, 0.0), (2.0, 0.0, -2.0)]
    graph_edges = [(2, 12), (3, 12), (2, 3), (7, 18), (2, 7), (3, 18), (5, 7), (5, 18)]
    graph = NavmeshGraph(graph_positions, graph_vertices, graph_edges)
    print(graph.search(3, 7))


def generate_grid_graph(n, m, limit=1.0):
    '''generate grid-like graph with randomly deleted edges

    this function used in the next example
    '''
    vertices = [m*i+j for i in range(n) for j in range(m)]
    positions = [(float(i), 0.0, float(j)) for i in range(n) for j in range(m)]
    edges = []
    for i in range(n):
        for j in range(m):
            if random.uniform(0.0, 1.0) < limit:  # ignore random edges of the grid
                if j < m - 1:
                    edges.append((m*i+j, m*i+j+1))
                if i < n - 1:
                    edges.append((m*i+j, m*(i+1)+j))

    # form the graph
    graph = NavmeshGraph(positions, vertices, edges)
    return graph


def grid_example():
    '''in this exmple we generate grid-like graph, find the shortest path between two random vertices and draw it by using matplotlib
    '''
    graph = generate_grid_graph(10, 10, limit=0.75)
    # get two random vertices
    start = random.randint(0, graph.get_vertex_count() - 1)
    end = random.randint(0, graph.get_vertex_count() - 1)
    path = graph.search(start, end)
    print("start vertex: " + str(start) + ", end vertex: " + str(end) + ", shortest path: " + str(path))

    # draw in plot
    plot_points = [(p[0], p[2]) for p in graph.get_positions()]
    [p_x, p_y] = zip(*plot_points)
    ax = plt.gca()
    ax.cla()
    ax.axis('equal')
    for edge in graph.get_edges():
        plt.plot([plot_points[edge[0]][0], plot_points[edge[1]][0]], [plot_points[edge[0]][1], plot_points[edge[1]][1]], 'g-', linewidth=2)
    ax.plot(p_x, p_y, 'go', markersize=4)

    # then plot the path
    path_x = [plot_points[v][0] for v in path]
    path_y = [plot_points[v][1] for v in path]
    ax.plot(path_x, path_y, 'r-', markersize=2, linewidth=1)

    # also polt start and end points in any case
    ax.plot([plot_points[start][0], plot_points[end][0]], [plot_points[start][1], plot_points[end][1]], 'ro', markersize=4)
    plt.show()


def grid_benchmark():
    '''try to find the shortest path in large grid-like graph
    '''
    graph = generate_grid_graph(500, 100, 0.75)
    start, end = 0, graph.get_vertex_count() - 1
    start_time = time.time()
    path = graph.search(start, end)
    finish_time = time.time()
    print(start, end, path)
    print("calculation time: " + str(finish_time - start_time) + " seconds")


if __name__ == "__main__":
    import matplotlib.pyplot as plt  # type: ignore
    import time
    import random
    grid_example()
