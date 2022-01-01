import math
from typing import List, Tuple, Optional, Dict


class NavmeshNode:
    '''Class for representing one node in the navigation mesh. Each node is a polygon with some additional data:
        -) group index - the index of the connected component of polygons
        -) polygon index
        -) array of indexes of incident polygons
        -) indexes of polygon vertices
        -) coordinates of the polygon center
        -) portals (this is a map, where key - index of the neighbor node, value - the pair of 3-tuples ((v1), (v2)), where v1 and v2 coordinates of the edge vertices )
                    ordered in orientetion, induced from polygon orientation

        polygons are convex, it's important for calculation it normal

        this class is for internal use only
    '''
    def __init__(self, all_vertices: List[Tuple[float, float, float]], index: int, polygon_indexes: List[int]):
        '''Pass to the constructor all vertices of the navigation mesh
        '''
        self._vertices: List[Tuple[float, float, float]] = [all_vertices[i] for i in polygon_indexes]
        self._polygon: List[int] = polygon_indexes  # indexes of polygon vertices
        self._group: int = -1
        self._index: int = index
        self._neighbors: List[int] = []
        self._center: Tuple[float, float, float] = self._calc_center(self._vertices)
        self._normal: Tuple[float, float, float] = self._calc_average_normal(self._center, self._vertices)
        self._portals: Dict[int, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = {}
        self._vertex_normals: List[Tuple[float, float, float]] = self._calc_vertex_normals(self._vertices)

    def add_neighbor(self, node_index: int, vertex_0: Tuple[float, float, float], vertex_1: Tuple[float, float, float]):
        '''add neighbor polygon and portal
        '''
        if node_index not in self._neighbors:
            self._neighbors.append(node_index)
            self._portals[node_index] = (vertex_0, vertex_1)
        else:
            print("[Something wrong] Try to add neighbor node " + str(node_index) + " to the " + str(self._index) + ", but this node already exists")

    def get_polygon(self) -> List[int]:
        return self._polygon

    def get_portal(self, node_index: int) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        '''return pair of 3-tuples with coordinates of the edge, which separate current node from node with index node_index
        '''
        return self._portals[node_index]

    def get_neighbord(self) -> List[int]:
        return self._neighbors

    def get_index(self) -> int:
        return self._index

    def get_group(self) -> int:
        return self._group

    def get_vertex_indexes(self) -> List[int]:
        return self._polygon

    def get_vertex_coordinates(self) -> List[Tuple[float, float, float]]:
        return self._vertices

    def get_normal(self) -> Tuple[float, float, float]:
        return self._normal

    def get_center(self) -> Tuple[float, float, float]:
        return self._center

    def set_group(self, group_index: int, group_array: List[int], all_nodes):  # all_nodes: List[NavmeshNode]
        if self._group == -1:
            self._group = group_index
            group_array.append(self._index)
            for n in self._neighbors:
                all_nodes[n].set_group(group_index, group_array, all_nodes)

    def is_point_inside(self, point: Tuple[float, float, float]) -> bool:
        '''return true, if the point inside the polygon
        '''
        size: int = len(self._vertices)
        for i in range(size):
            u: Tuple[float, float, float] = self._vertices[i]
            v: Tuple[float, float, float] = self._vertices[self._modulize(i + 1, size)]
            # we should calculate cross product [uv, up]
            # if dot-product with normal in the vertex < 0, then point outside the polygon
            vector: Tuple[float, float, float] = self._cross(v[0] - u[0], v[1] - u[1], v[2] - u[2], point[0] - u[0], point[1] - u[1], point[2] - u[2])
            d: float = vector[0] * self._vertex_normals[i][0] + vector[1] * self._vertex_normals[i][1] + vector[2] * self._vertex_normals[i][2]
            if d < 0.0:
                return False
        return True

    def _modulize(self, value: int, n: int) -> int:
        if value < n:
            return value
        else:
            return value - n

    def _calc_vertex_normals(self, vertices: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        '''for each vertex u (and next v, w) calculate normalized [uv, uw]
        '''
        size: int = len(vertices)
        to_return: List[Tuple[float, float, float]] = []
        for i in range(size):
            u: Tuple[float, float, float] = vertices[i]
            v: Tuple[float, float, float] = vertices[self._modulize(i + 1, size)]
            w: Tuple[float, float, float] = vertices[self._modulize(i + 2, size)]
            vector: Tuple[float, float, float] = self._cross(v[0] - u[0], v[1] - u[1], v[2] - u[2], w[0] - u[0], w[1] - u[1], w[2] - u[2])
            d: float = math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)
            to_return.append((vector[0] / d, vector[1] / d, vector[2] / d))
        return to_return

    def _calc_center(self, array: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        '''Find center of the array
        '''
        s = [0.0] * 3
        for a in array:
            for i in range(len(a)):
                s[i] += a[i]
        for i in range(len(s)):
            s[i] = s[i] / len(array)
        return (s[0], s[1], s[2])

    def _cross(self, a_x: float, a_y: float, a_z: float, b_x: float, b_y: float, b_z: float) -> Tuple[float, float, float]:
        return (a_y*b_z - a_z*b_y, a_z*b_x - a_x*b_z, a_x*b_y - a_y*b_x)

    def _calc_average_normal(self, center: Tuple[float, float, float], points: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        normal: List[float] = [0.0, 0.0, 0.0]
        for p_index in range(len(points)):
            p0: Tuple[float, float, float] = points[p_index]
            p1: Tuple[float, float, float] = points[p_index + 1 if p_index < len(points) - 1 else 0]
            c: Tuple[float, float, float] = self._cross(p0[0] - center[0], p0[1] - center[1], p0[2] - center[2], p1[0] - center[0], p1[1] - center[1], p1[2] - center[2])
            normal[0] += c[0]
            normal[1] += c[1]
            normal[2] += c[2]
        d: float = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
        return (normal[0] / d, normal[1] / d, normal[2] / d)

    def __repr__(self):
        return "<node " + str(self._index) +\
               ", vertices: " + str(self._polygon) +\
               ", normal: " + str(self._normal) +\
               ", local normals: " + str(self._vertex_normals) +\
               ", neigh: " + str(self._neighbors) +\
               ", component: " + str(self._group) +\
               ", portals: " + str(self._portals) +\
               ">"
