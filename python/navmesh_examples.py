from navmesh import Navmesh
from typing import List, Tuple


def generate_grid(n: int, m: int, grid_size: float = 1.0) -> Navmesh:
    '''generate navmesh for the grid n x m
    '''
    vertices: List[Tuple[float, float, float]] = []
    for i in range(n):
        for j in range(m):
            vertices.append((i*grid_size, 0.0, j*grid_size))
    polygons: List[List[int]] = []
    for i in range(n - 1):
        for j in range(m - 1):
            polygons.append([m * i + j, m * (i + 1) + j, m * (i + 1) + j + 1, m * i + j + 1])
    navmesh: Navmesh = Navmesh(vertices, polygons)
    return navmesh

def generate_from_file(file_path: str) -> Navmesh:
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
        navmesh = Navmesh(vertices, polygons)
    return navmesh


if __name__ == "__main__":
    navmesh = generate_from_file("level.txt")
    start = (1.0, 0.0, -2.0)
    finish = (1.0, 0.0, 2.0)
    path = navmesh.serach_path(start, finish)
    print(path)
