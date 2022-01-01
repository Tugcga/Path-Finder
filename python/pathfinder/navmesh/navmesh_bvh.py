from typing import List, Tuple, Optional
from pathfinder.navmesh.navmesh_node import NavmeshNode

BVH_AABB_DELTA = 0.5


class NavmeshBVH:
    '''Class for the node int bvh-tree
    '''
    def __init__(self, nodes: List[NavmeshNode]):
        '''Create one node (and call recursive building all children nodes) from array of NavmeshNodes

        Input:
            nodes - array of NavmeshNode objects
        '''
        self._node: Optional[NavmeshNode] = None
        self._left: Optional[NavmeshBVH] = None
        self._right: Optional[NavmeshBVH] = None

        x_min: float = float("inf")
        x_max: float = -float("inf")
        y_min: float = float("inf")
        y_max: float = -float("inf")
        z_min: float = float("inf")
        z_max: float = -float("inf")
        self._aabb: Tuple[float, float, float, float, float, float]

        if len(nodes) == 1:
            self._node = nodes[0]
            # build aabb
            verts: List[Tuple[float, float, float]] = self._node.get_vertex_coordinates()
            for v in verts:
                if v[0] < x_min:
                    x_min = v[0]
                if v[0] > x_max:
                    x_max = v[0]
                if v[1] < y_min:
                    y_min = v[1]
                if v[1] > y_max:
                    y_max = v[1]
                if v[2] < z_min:
                    z_min = v[2]
                if v[2] > z_max:
                    z_max = v[2]
            self._aabb = (x_min - BVH_AABB_DELTA, y_min - BVH_AABB_DELTA, z_min - BVH_AABB_DELTA, x_max + BVH_AABB_DELTA, y_max + BVH_AABB_DELTA, z_max + BVH_AABB_DELTA)
        else:
            # find the axis (x or z) to split the space
            x_median: float = 0.0
            z_median: float = 0.0
            for node in nodes:
                c: Tuple[float, float, float] = node.get_center()
                x_median += c[0]
                z_median += c[2]
                if c[0] < x_min:
                    x_min = c[0]
                if c[0] > x_max:
                    x_max = c[0]
                if c[2] < z_min:
                    z_min = c[2]
                if c[2] > z_max:
                    z_max = c[2]
            split_axis: int = 0 if (x_max - x_min) > (z_max - z_min) else 2
            median: float = x_median / len(nodes) if (x_max - x_min) > (z_max - z_min) else z_median / len(nodes)
            left: List[NavmeshNode] = []
            right: List[NavmeshNode] = []
            for node in nodes:
                if node.get_center()[split_axis] < median:
                    left.append(node)
                else:
                    right.append(node)
            if len(left) == 0:
                # move last right node to the left array
                left.append(right.pop())
            else:
                # left array is not empty, but may be empty right array
                if len(right) == 0:
                    right.append(left.pop())
            self._left = NavmeshBVH(left)
            self._right = NavmeshBVH(right)
            l_aabb: Tuple[float, float, float, float, float, float] = self._left.get_aabb()
            r_aabb: Tuple[float, float, float, float, float, float] = self._right.get_aabb()
            self._aabb = self._union_aabbs(l_aabb, r_aabb)

    def _union_aabbs(self, b1: Tuple[float, float, float, float, float, float], b2: Tuple[float, float, float, float, float, float]) -> Tuple[float, float, float, float, float, float]:
        return (min(b1[0], b2[0]), min(b1[1], b2[1]), min(b1[2], b2[2]),
                max(b1[3], b2[3]), max(b1[4], b2[4]), max(b1[5], b2[5]))

    def get_aabb(self) -> Tuple[float, float, float, float, float, float]:
        '''Return 6-tuple of the aabb (axis align bounding box) of the current node

        this 6-tuple is (x_min, y_min, z_min, x_max, y_max, z_max)
        '''
        return self._aabb

    def is_inside_aabb(self, point: Tuple[float, float, float]) -> bool:
        '''Return True, if the point is inside the current aabb, and False otherwise

        Input:
            point - 3-triple (x, y, z)

        Output:
            True or False
        '''
        return self._aabb[0] < point[0] and self._aabb[1] < point[1] and self._aabb[2] < point[2] and\
               self._aabb[3] > point[0] and self._aabb[4] > point[1] and self._aabb[5] > point[2]

    def sample(self, point: Tuple[float, float, float]) -> Optional[NavmeshNode]:
        '''Return node, which contains the point
        If there are no nodes near the point, then return None

        Input:
            point - 3-triple (x, y, z)
        
        Output:
            NavmeshNode or None
        '''
        if self.is_inside_aabb(point):
            if self._node is None and self._left is not None and self._right is not None:
                # this bvh node contains left and right children, go deeper
                left_sample: Optional[NavmeshNode] = self._left.sample(point)
                right_sample: Optional[NavmeshNode] = self._right.sample(point)
                if left_sample is None:
                    return right_sample
                else:
                    if right_sample is None:
                        return left_sample
                    else:
                        # we should choose from left and right sample the closest to the point
                        l_c: Tuple[float, float, float] = left_sample.get_center()
                        l_n: Tuple[float, float, float] = left_sample.get_normal()
                        l_dist: float = abs((point[0] - l_c[0]) * l_n[0] + (point[1] - l_c[1]) * l_n[1] + (point[2] - l_c[2]) * l_n[2])

                        r_c: Tuple[float, float, float] = right_sample.get_center()
                        r_n: Tuple[float, float, float] = right_sample.get_normal()
                        r_dist: float = abs((point[0] - r_c[0]) * r_n[0] + (point[1] - r_c[1]) * r_n[1] + (point[2] - r_c[2]) * r_n[2])

                        if l_dist < r_dist:
                            return left_sample
                        else:
                            return right_sample
            else:
                # bvh node contains polygon
                if self._node is not None:
                    if self._node.is_point_inside(point):
                        return self._node
                    else:
                        return None
                else:
                    return None
        else:
            return None

    def __repr__(self) -> str:
        return "<object: " + str(self._node if self._node is None else self._node.get_index()) + ", left: " + str(self._left) + ", right: " + str(self._right) + ">"
