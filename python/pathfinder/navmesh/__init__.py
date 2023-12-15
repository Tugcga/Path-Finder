import math
from typing import List, Tuple, Optional
from pathfinder.navmesh.navmesh_graph import NavmeshGraph
from pathfinder.navmesh.navmesh_node import NavmeshNode
from pathfinder.navmesh.navmesh_bvh import NavmeshBVH
from pathfinder.navmesh.navmesh_triangle import TrianglesBVH, polygons_to_triangles


class Navmesh:
    def __init__(self, vertices: List[Tuple[float, float, float]], polygons: List[List[int]]):
        self._vertices: List[Tuple[float, float, float]] = vertices
        self._polygons: List[List[int]] = polygons
        self._graphs: List[NavmeshGraph] = []  # graph, where vertices are centers of polygons, edges are pairs of two incident (by edge only!) polygons
        self._nodes: List[NavmeshNode] = []
        self._groups: List[List[int]] = []  # each group is an array of node indexes with the same group index

        vertex_map: List[List[int]] = [[] for v in range(len(vertices))]  # index - vertex index, value - array of incident polygons
        for p_index in range(len(polygons)):
            # iterate throw polygons
            for p_v_index in polygons[p_index]:
                # add polygon index to the array of corresponding vertices
                vertex_map[p_v_index].append(p_index)
            # create the node
            self._nodes.append(NavmeshNode(vertices, p_index, polygons[p_index]))

        # next we can use vertex_map for define neighbors of each polygon
        for node in self._nodes:
            node_vertices: List[int] = node.get_vertex_indexes()
            # for each vertex of the node (polygon) we should get indexes of incident polygons and take intersections of each sequential pair
            for v_num in range(len(node_vertices)):
                u: int = node_vertices[v_num]
                v: int = node_vertices[v_num + 1 if v_num < len(node_vertices) - 1 else 0]
                intersection: List[int] = self._get_intersection(vertex_map[u], vertex_map[v])
                if len(intersection) == 0:
                    print("[Something wrong] Intersection of polygons, incident to vertices " + str(u) + " and " + str(v) + " are empty")
                elif len(intersection) == 2:
                    # in principle, other cases are impossible
                    node_index: int = node.get_index()
                    if node_index not in intersection:
                        print("[Something wrong] Polygon " + str(node_index) + " does not contained in the neighborhood of two incident vertices")
                    else:
                        for i in intersection:
                            if i != node_index:
                                node.add_neighbor(i, self._vertices[u], self._vertices[v])
                elif len(intersection) > 2:
                    print("[Something wrong] Intersection of polygons, incident to vertices " + str(u) + " and " + str(v) + " contains " + str(len(intersection)) + " items " + str(intersection))

        # define groups
        for node in self._nodes:
            g: int = node.get_group()
            if g == -1:
                # if we get the first polygon with undefined group
                new_group: List[int] = []  # start new group array
                new_index: int = len(self._groups)  # generate the next group index
                node.set_group(new_index, new_group, self._nodes)  # start recursive provess
                self._groups.append(new_group)

        # finally, define the graph
        # one graph for each group
        for group in self._groups:
            graph_edges: List[Tuple[int, int]] = []  # each graph is an aray of pairs (n1, n2), where n1 and n2 are node (=polygon) indexes and n1 < n2
            for node_index in group:
                node_neighbors: List[int] = self._nodes[node_index].get_neighbord()
                for other_node in node_neighbors:
                    edge: Tuple[int, int] = (node_index, other_node) if node_index < other_node else (other_node, node_index)
                    # if this edge is new, add it to the graph
                    if edge not in graph_edges:
                        graph_edges.append(edge)
            # next we should get all graph node indexes, sort it and get it centers
            graph_verts: List[int] = []
            for edge in graph_edges:
                for i in range(2):
                    if edge[i] not in graph_verts:
                        graph_verts.append(edge[i])
            graph_verts.sort()
            self._graphs.append(NavmeshGraph([self._nodes[i].get_center() for i in graph_verts], graph_verts, graph_edges))

        # build bvh
        self._bvh: NavmeshBVH = NavmeshBVH(self._nodes)

        # build triangles bvh
        triangles = polygons_to_triangles(vertices, polygons)
        self._triangles_bvh: TrianglesBVH = TrianglesBVH(triangles)

    def get_groups_count(self) -> int:
        '''Return the number of polygon groups (connected components) in the navigation mesh
        '''
        return len(self._groups)

    def get_group_polygons(self, group_index: int) -> List[List[int]]:
        '''return polygons for the given group in the form [[i11, i12, ..., i1n], [i21, i22, ...], ...]
        '''
        if group_index < len(self._groups):
            group: List[int] = self._groups[group_index]
            polygons: List[List[int]] = []
            for p_index in group:
                polygons.append(self._nodes[p_index].get_polygon())
            return polygons
        else:
            return []

    def sample_polygon(self, position: Tuple[float, float, float]) -> Optional[NavmeshNode]:
        '''return node, close to the given point, or None, if it outside of the navmesh
        '''
        return self._bvh.sample(position)

    def sample(self, point: Tuple[float, float, float], is_slow: bool = False) -> Optional[Tuple[float, float, float]]:
        '''return coordinates of the point inside navmesh, closest to the input one
        if it fails to find the closest point, return None

        Input:
            point - coordinates of the input point
            is_slow - set True to find closest point in all triangles of the navmesh,
                      set False to use BVH and skip triangles in bounding boxes far away from the input point

        Output:
            coordinates of the closest point or None
        '''
        return self._triangles_bvh.sample(point, is_slow)

    def raycast(self, origin: Tuple[float, float, float], direction: Tuple[float, float, float]) -> Optional[Tuple[float, float, float]]:
        '''return coordinates of the intersection of the navigation mesh and the ray with a given origina and direction
        if navmesh is not defined then return None

        Input:
            origin - coordinates of the point in the ray
            direction - coordinates of the ray direction

        Output:
            coordinates of the intersection point or None, if the ray does not intersect the navigation mesh
        '''
        return self._triangles_bvh.raycast(origin, direction)

    def search_path(self,
                    start: Tuple[float, float, float],
                    finish: Tuple[float, float, float],
                    length_limit_coefficient: Optional[float] = None) -> List[Tuple[float, float, float]]:
        '''Search the path between start and finish points in the navigation mesh

        Input:
            start - 3-tuple (x, y, z) of the start point
            finish - 3-tuple (x, y, z) of the finish point
            length_limit_coefficient - float or None (by default), shoulw be >= 1.0

        Return:
            array in the form [(x1, y1, z1), (x2, y2, z2), ...] with coordinates of points, which form the output path
            start and finish points include as the first and the last entries in the array
            if there is no path between input points, then return empty array []
            if parameter length_limit_coefficient is not None, then search all pathes between start and end point
            with length between shortest value and value, obtained by multiplication to the parameter
            in some cases this allows to find more short path (it may corresponds not shortest path in the graph)
            use parameter length_limit_coefficient carefully, because it can leads to the combinatorial explosion
        '''
        # find nodes indexes for start and end point
        start_node: Optional[NavmeshNode] = self._bvh.sample(start)
        finish_node: Optional[NavmeshNode] = self._bvh.sample(finish)
        if start_node is not None and finish_node is not None:
            # check are nodes in one group
            start_index: int = start_node.get_index()
            finish_index: int = finish_node.get_index()
            group_index: int = self._get_nodes_group_index(start_index, finish_index)
            if group_index > -1:
                graph: NavmeshGraph = self._graphs[group_index]
                # find path between nodes in the graph
                graph_min_path: List[int] = graph.search(start_index, finish_index)

                # next all pathes in the graph with allowed length
                graph_collects = [graph_min_path] if length_limit_coefficient is None else graph.collect_pathes(graph_min_path, length_limit_coefficient)
                to_return = []
                min_length = float("inf")
                for graph_path in graph_collects:
                    # next create non-optimal path throw portals
                    raw_path: List[Tuple[float, float, float]] = [start, start]
                    for p_i in range(1, len(graph_path)):
                        # extend raw path by portal points between p_i-th node and p_i+1-th
                        portal: Tuple[Tuple[float, float, float], Tuple[float, float, float]] = self._nodes[graph_path[p_i - 1]].get_portal(graph_path[p_i])
                        raw_path.extend(portal)
                    raw_path.extend([finish, finish])

                    # finally, simplify the raw_path, by using pull the rope algorithm
                    # get it from https://github.com/donmccurdy/three-pathfinding
                    portal_apex: Tuple[float, float, float] = raw_path[0]
                    portal_left: Tuple[float, float, float] = raw_path[0]
                    portal_right: Tuple[float, float, float] = raw_path[1]

                    apex_index: int = 0
                    left_index: int = 0
                    right_index: int = 0

                    finall_path: List[Tuple[float, float, float]] = [portal_apex]
                    i: int = 1
                    while i < len(raw_path) // 2:
                        left: Tuple[float, float, float] = raw_path[2 * i]
                        right: Tuple[float, float, float] = raw_path[2 * i + 1]

                        skip_next: bool = False
                        # update right vertex
                        if self._triangle_area_2(portal_apex, portal_right, right) <= 0.0:
                            if self._v_equal(portal_apex, portal_right) or self._triangle_area_2(portal_apex, portal_left, right) > 0.0:
                                portal_right = right
                                right_index = i
                            else:
                                if not self._v_equal(portal_left, finall_path[-1]):
                                    finall_path.append(portal_left)
                                # make current left the new apex
                                portal_apex = portal_left
                                apex_index = left_index
                                # reset portal
                                portal_left = portal_apex
                                portal_right = portal_apex
                                left_index = apex_index
                                right_index = apex_index
                                # restart scan
                                i = apex_index
                                skip_next = True
                        if not skip_next:
                            # update left vertex
                            if self._triangle_area_2(portal_apex, portal_left, left) >= 0.0:
                                if self._v_equal(portal_apex, portal_left) or self._triangle_area_2(portal_apex, portal_right, left) < 0.0:
                                    portal_left = left
                                    left_index = i
                                else:
                                    finall_path.append(portal_right)
                                    # make current right the new apex
                                    portal_apex = portal_right
                                    apex_index = right_index
                                    # reset portal
                                    portal_left = portal_apex
                                    portal_right = portal_apex
                                    left_index = apex_index
                                    right_index = apex_index
                                    # restart scan
                                    i = apex_index
                        i += 1
                    if (len(finall_path) == 0 or not self._v_equal(finall_path[len(finall_path) - 1], raw_path[len(raw_path) - 2])):
                        # append last point to path
                        finall_path.append(raw_path[len(raw_path) - 2])
                    # calculate the length of the finall path in this iteration
                    path_length = self._get_path_length(finall_path)
                    if path_length < min_length:
                        to_return = finall_path
                        min_length = path_length

                return to_return
            else:
                # nodes in the different groups, so, there are no path between them
                return []
        else:
            # start or finish node is None, so, no path
            return []

    def _get_path_length(self, path: List[Tuple[float, float, float]]) -> float:
        if len(path) == 0:
            return 0.0
        point = path[0]
        length = 0.0
        for i in range(1, len(path)):
            next_point = path[i]
            add_length = math.sqrt((next_point[0] - point[0])**2 + (next_point[1] - point[1])**2 + (next_point[2] - point[2])**2)
            length += add_length
            point = next_point
        return length

    def _v_equal(self, a: Tuple[float, float, float], b: Tuple[float, float, float], epsilon: float = 0.0001) -> bool:
        '''a, b are points

        return True, if distance from a to b less, then epsilon
        '''
        return (a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2 < epsilon

    def _triangle_area_2(self, a: Tuple[float, float, float], b: Tuple[float, float, float], c: Tuple[float, float, float]) -> float:
        '''a, b, and c are 3-tuples with vertex coordinates
        '''
        return (c[0] - a[0]) * (b[2] - a[2]) - (b[0] - a[0]) * (c[2] - a[2])

    def _check_nodes_in_one_group(self, index_01: int, index_02: int) -> bool:
        for group in self._groups:
            if index_01 in group and index_02 in group:
                return True
        return False

    def _get_nodes_group_index(self, index_01: int, index_02: int) -> int:
        '''return index of the group with indexes index_01 and index_02, -1 if these values are in different groups
        '''
        for i in range(len(self._groups)):
            group: List[int] = self._groups[i]
            if index_01 in group and index_02 in group:
                return i
        return -1

    def _get_intersection(self, array_a: List[int], array_b: List[int]) -> List[int]:
        '''return intersection of two arrays
        '''
        to_return: List[int] = []
        for a in array_a:
            if a in array_b:
                to_return.append(a)
        return to_return
