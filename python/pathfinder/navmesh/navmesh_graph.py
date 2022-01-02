import math
from typing import List, Tuple, Dict


class NavmeshGraph:
    '''Graph with specific data, needed for A* algorithm
    '''
    def __init__(self, vertex_positions: List[Tuple[float, float, float]], vertices: List[int], edges: List[Tuple[int, int]]):
        '''Inicialize the graph from it vertices and edges

        Input:
            vertex_positions - array of 3-tuples [(x, y, z), ...]
            vertices - array of integer names of the vertices
            edges - array of 2-tuple [(a, b), ...], where a and b are names of vertices, incident to the same edge
        '''
        self._is_empty = len(vertex_positions) == 0
        self._positions: List[Tuple[float, float, float]] = vertex_positions
        self._vertex_names: List[int] = vertices  # names are not from 0 to n-1
        self._vertex_count: int = len(self._vertex_names)
        # we need oppisit map from vertex name to it index
        self._index_map: Dict[int, int] = {}
        for i in range(len(self._vertex_names)):
            self._index_map[self._vertex_names[i]] = i
        self._edges: List[Tuple[int, int]] = edges

        # create map from vertex to all incident vertices
        self._incident_map: Dict[int, List[int]] = {}  # key - vertex index, value - array of indexes of incident vertices
        for e in self._edges:
            for i in range(2):
                v = self._index_map[e[i]]
                if v in self._incident_map:
                    self._incident_map[v].append(self._index_map[e[i-1]])
                else:
                    self._incident_map[v] = [self._index_map[e[i-1]]]

    def _pre_start(self, target: int):
        self._vertices_g: List[float] = [0.0] * self._vertex_count
        self._vertices_h: List[float] = [self._get_distance(self._positions[i], self._positions[target]) for i in range(self._vertex_count)]  # calculate distance from each vertex to the target
        self._vertices_f: List[float] = [0.0] * self._vertex_count
        self._vertices_close: List[bool] = [False] * self._vertex_count
        self._vertices_parent: List[int] = [-1] * self._vertex_count  # store here the index of the vertex, from we come here with minimum g

    def _get_distance(self, a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

    def get_vertex_count(self):
        return self._vertex_count

    def get_vertices(self):
        '''Return graph vertex names
        '''
        return self._vertex_names

    def get_positions(self):
        '''Return the array of graph vertex positions
        '''
        return self._positions

    def get_edges(self):
        '''Return array of graph edges
        '''
        return self._edges

    def search(self, start_vertex: int, end_vertex: int) -> List[int]:
        '''Calculate the shortes path between two input vertices, by using A* algorithm

        Input:
            start_vertex - the name of the start vertex
            end_vertex - the name of the end vertex

        Output:
            array of vertex names, which form the shortest path between start end end vertices
            if there are no path between input vertices, then return empty array []
        '''
        if self._is_empty:
            return []
        # clear data structures
        start: int = self._index_map[start_vertex]
        end: int = self._index_map[end_vertex]
        self._pre_start(end)
        # create open list
        open_list: List[int] = [start]
        # set f and g for the start_vertex
        self._vertices_g[start] = 0.0
        self._vertices_f[start] = self._vertices_g[start] + self._vertices_h[start]

        while len(open_list) > 0:
            # find in the open list vertex with the minimum value of f
            min_f: float = self._vertices_f[open_list[0]]
            min_vertex: int = open_list[0]
            min_index: int = 0
            v: int
            for i in range(1, len(open_list)):
                v = open_list[i]
                v_f: float = self._vertices_f[v]
                if v_f < min_f:
                    min_f = v_f
                    min_vertex = v
                    min_index = i
            open_list.pop(min_index)  # remove vertex with minimal f from the open list
            self._vertices_close[min_vertex] = True  # and set it close
            if min_vertex == end:
                # we find the closest path
                path: List[int] = []  # put to the path indexes of vertices
                v = min_vertex
                while self._vertices_parent[v] > -1:
                    path.append(v)
                    v = self._vertices_parent[v]
                path.append(v)
                # return array of vertices names
                l: int = len(path)
                return [self._vertex_names[path[l - 1 - i]] for i in range(l)]
            # next we should enumerate all vertex edges and add all non-initial vertices to the open list
            if min_vertex in self._incident_map:
                for child_index in self._incident_map[min_vertex]:
                    if self._vertices_close[child_index] is False:
                        v_g: float = self._get_distance(self._positions[min_vertex], self._positions[child_index]) + self._vertices_g[min_vertex]
                        if self._vertices_parent[child_index]  == -1:
                            # we come to this vertex at first time, so, set parent and g-value
                            self._vertices_parent[child_index] = min_vertex
                            self._vertices_g[child_index] = v_g
                            self._vertices_f[child_index] = v_g + self._vertices_h[child_index]
                            # add it to the open list
                            open_list.append(child_index)
                        else:
                            # we already come to this vertex, so, this vertex in the open list
                            # check, may be g-value is less, than existing one
                            if self._vertices_g[child_index] > v_g:
                                # redefine parent
                                self._vertices_parent[child_index] = min_vertex
                                self._vertices_g[child_index] = v_g
                                self._vertices_f[child_index] = v_g + self._vertices_h[child_index]
        # empty array mean, that there are no path between vertices in the graph
        return []

    def __repr__(self):
        return "<graph " + str(self._vertex_names) + ", edges: " + str(self._edges) + ", map: " + str(self._index_map) + ", positions: " + str(self._positions) + ">"
