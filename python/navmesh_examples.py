from pathfinder import PathFinder
from typing import List, Tuple


def generate_grid(n: int, m: int, grid_size: float = 1.0, ignore_cell=-1) -> PathFinder:
    '''generate navmesh for the grid n x m

    optional, may delete one grid cell
    '''
    vertices: List[Tuple[float, float, float]] = []
    for i in range(n):
        for j in range(m):
            vertices.append((i*grid_size, 0.0, j*grid_size))
    polygons: List[List[int]] = []
    polygon_index: int = 0
    for i in range(n - 1):
        for j in range(m - 1):
            if polygon_index != ignore_cell:
                polygons.append([m * i + j, m * i + j + 1, m * (i + 1) + j + 1, m * (i + 1) + j])
            polygon_index += 1
    navmesh: PathFinder = PathFinder(vertices, polygons)
    return navmesh

def generate_from_file(file_path: str) -> PathFinder:
    '''open text file from the file_path and generate navmesh by using vertices and polygons from this file

    the sintaxis of the file is the following: it containt two strings, the first string contains vertex coordinates, devided by " " (space)
    the second string contains triangle vertex indexes, also devided by " "(space)

    so, all nodes in the builded navmesh are triangles
    '''
    navmesh = None
    with open(file_path, "r") as file:
        text = file.read()
        parts = text.split("\n")
        vertices_values = [eval(v) for v in parts[0].split(" ")]
        vertices = [(vertices_values[3*i], vertices_values[3*i + 1], vertices_values[3*i + 2]) for i in range(len(vertices_values) // 3)]
        polygon_indexes = [eval(v) for v in parts[1].split(" ")]
        # split by 3 triangles
        polygons = [polygon_indexes[3*i:3*(i+1)] for i in range(len(polygon_indexes) // 3)]
        navmesh = PathFinder(vertices, polygons)
    return navmesh


def example_grid():
    navmesh = generate_grid(3, 3, ignore_cell=1)
    start = (0.0, 0.0, 0.5)
    finish = (1.5, 0.0, 2.0)
    path = navmesh.search_path(start, finish)
    print(path)


def example_from_file():
    navmesh = generate_from_file("level_triangles.txt")
    start = (-2.0, 0.0, -2.0)
    finish = (2.0, 0.0, 2.0)
    path = navmesh.search_path(start, finish)
    print(path)


if __name__ == "__main__":
    example_from_file()
