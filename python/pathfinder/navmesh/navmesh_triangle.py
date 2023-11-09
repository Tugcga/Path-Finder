from typing import List, Tuple, Optional


def clamp(a: float, min: float = 0.0, max: float = 1.0) -> float:
    if a < min:
        return min
    elif a > max:
        return max
    else:
        return a


class Triangle:
    def __init__(self, vertices: List[Tuple[float, float, float]]):
        self._v0: Tuple[float, float, float] = vertices[0]
        self._v1: Tuple[float, float, float] = vertices[1]
        self._v2: Tuple[float, float, float] = vertices[2]

        self._aabb = (min((self._v0[0], self._v1[0], self._v2[0])), min((self._v0[1], self._v1[1], self._v2[1])), min((self._v0[2], self._v1[2], self._v2[2])),
                      max((self._v0[0], self._v1[0], self._v2[0])), max((self._v0[1], self._v1[1], self._v2[1])), max((self._v0[2], self._v1[2], self._v2[2])))
        self._center = ((self._v0[0] + self._v1[0] + self._v2[0]) / 3.0,
                        (self._v0[1] + self._v1[1] + self._v2[1]) / 3.0,
                        (self._v0[2] + self._v1[2] + self._v2[2]) / 3.0)

        # we should store additional vectors
        # e1 = v0->v1
        self._e1 = (self._v1[0] - self._v0[0], self._v1[1] - self._v0[1], self._v1[2] - self._v0[2])
        # e2 = v0->v2
        self._e2 = (self._v2[0] - self._v0[0], self._v2[1] - self._v0[1], self._v2[2] - self._v0[2])
        # a = (e1, e1)
        self._a = self._e1[0] ** 2 + self._e1[1] ** 2 + self._e1[2] ** 2
        # b = (e1, e2)
        self._b = self._e1[0] * self._e2[0] + self._e1[1] * self._e2[1] + self._e1[2] * self._e2[2]
        # c = (e2, e2)
        self._c = self._e2[0] ** 2 + self._e2[1] ** 2 + self._e2[2] ** 2
        # det = det((a b) (b c)) = a * c - b**2
        self._det = self._a * self._c - self._b ** 2

    def get_aabb(self) -> Tuple[float, float, float, float, float, float]:
        '''Return AABB of the triangle

        in the form (x_min, y_min, z_min, x_max, y_max, z_max)
        '''
        return self._aabb

    def get_center(self) -> Tuple[float, float, float]:
        '''Return coordinates of the triangle center
        '''
        return self._center

    def get_closest_point(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        '''Return coordinates of the point inside triangle, closest to the input one

        Input:
            point - 3-tuple

        Output:
            3-tuple
        '''
        # get vector p->v0
        p_v0 = (self._v0[0] - point[0], self._v0[1] - point[1], self._v0[2] - point[2])
        # d = (e1, p->v0)
        d = self._e1[0] * p_v0[0] + self._e1[1] * p_v0[1] + self._e1[2] * p_v0[2]
        # e = (e2, p->v0)
        e = self._e2[0] * p_v0[0] + self._e2[1] * p_v0[1] + self._e2[2] * p_v0[2]
        # s = b*e - c*d
        s = self._b * e - self._c * d
        # t = b*d - a*e
        t = self._b * d - self._a * e

        if s + t < self._det:
            if s < 0.0:
                if t < 0.0:
                    if d < 0.0:
                        s = clamp(-d / self._a)
                        t = 0
                    else:
                        s = 0
                        t = clamp(-e / self._c)
                else:
                    s = 0
                    t = clamp(-e / self._c)
            elif t < 0.0:
                s = clamp(-d / self._a)
                t = 0.0
            else:
                inv_det: float = 1.0 / self._det
                s *= inv_det
                t *= inv_det
        else:
            if s < 0.0:
                tmp_0 = self._b + d
                tmp_1 = self._c + e
                if tmp_1 > tmp_0:
                    numer = tmp_1 - tmp_0
                    denom = self._a - 2 * self._b + self._c
                    s = clamp(numer / denom)
                    t = 1.0 - s
                else:
                    t = clamp(-e / self._c)
                    s = 0
            elif t < 0.0:
                if self._a + d > self._b + e:
                    numer = self._c + e - self._b - d
                    denom = self._a - 2 * self._b + self._c
                    s = clamp(numer / denom)
                    t = 1.0 - s
                else:
                    s = clamp(-d / self._a)
                    t = 0
            else:
                numer = self._c + e - self._b - d
                denom = self._a - 2 * self._b + self._c
                s = clamp(numer / denom)
                t = 1.0 - s
        return (self._v0[0] + s * self._e1[0] + t * self._e2[0],
                self._v0[1] + s * self._e1[1] + t * self._e2[1],
                self._v0[2] + s * self._e1[2] + t * self._e2[2])

    def __repr__(self):
        return "[" + str(self._v0) + ", " + str(self._v1) + ", " + str(self._v2) + "]"


class TrianglesBVH:
    def __init__(self, triangles: List[Triangle], aabb_delta=0.5):
        self._triangle: Optional[Triangle] = None
        self._left: Optional[TrianglesBVH] = None
        self._right: Optional[TrianglesBVH] = None

        self._aabb: Tuple[float, float, float, float, float, float]
        if len(triangles) == 1:
            # this is a leaf node
            # get triangle aabb and add delta
            t = triangles[0]
            t_aabb = t.get_aabb()
            self._aabb = (t_aabb[0] - aabb_delta, t_aabb[1] - aabb_delta, t_aabb[2] - aabb_delta,
                          t_aabb[3] + aabb_delta, t_aabb[4] + aabb_delta, t_aabb[5] + aabb_delta)

            # set node data
            self._triangle = t
        else:
            # find the axis to split triangles array
            x_min: float = float("inf")
            x_max: float = -float("inf")
            z_min: float = float("inf")
            z_max: float = -float("inf")
            x_median: float = 0.0
            z_median: float = 0.0
            for t in triangles:
                t_center: Tuple[float, float, float] = t.get_center()
                x_median += t_center[0]
                z_median += t_center[2]
                if t_center[0] < x_min:
                    x_min = t_center[0]
                if t_center[0] > x_max:
                    x_max = t_center[0]
                if t_center[2] < z_min:
                    z_min = t_center[2]
                if t_center[2] > z_max:
                    z_max = t_center[2]
            split_axis: int = 0 if (x_max - x_min) > (z_max - z_min) else 2
            median: float = x_median / len(triangles) if (x_max - x_min) > (z_max - z_min) else z_median / len(triangles)

            left: List[Triangle] = []
            right: List[Triangle] = []
            for t in triangles:
                if t.get_center()[split_axis] < median:
                    left.append(t)
                else:
                    right.append(t)
            if len(left) == 0:
                left.append(right.pop())
            else:
                if len(right) == 0:
                    right.append(left.pop())
            # setup left and right nodes
            self._left = TrianglesBVH(left, aabb_delta)
            self._right = TrianglesBVH(right, aabb_delta)
            l_aabb: Tuple[float, float, float, float, float, float] = self._left.get_aabb()
            r_aabb: Tuple[float, float, float, float, float, float] = self._right.get_aabb()
            self._aabb = self._union_aabbs(l_aabb, r_aabb)

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

    def _union_aabbs(self, b1: Tuple[float, float, float, float, float, float],
                     b2: Tuple[float, float, float, float, float, float]) -> Tuple[float, float, float, float, float, float]:
        return (min(b1[0], b2[0]), min(b1[1], b2[1]), min(b1[2], b2[2]),
                max(b1[3], b2[3]), max(b1[4], b2[4]), max(b1[5], b2[5]))

    def sample(self, point: Tuple[float, float, float], is_slow=False) -> Optional[Tuple[float, float, float]]:
        '''Return coordinates of the point, closest to the input one
        return None if it fails to find the closest point

        Input:
            point - 3-triple (x, y, z)
            is_slow - True/Flase, if True then check all triangles, is False use aabb to skip tree branches

        Output:
            3-tuple or None
        '''
        if self.is_inside_aabb(point) or is_slow:
            if self._triangle is not None:
                return self._triangle.get_closest_point(point)
            else:
                if self._left is not None and self._right is not None:
                    left_sample: Optional[Tuple[float, float, float]] = self._left.sample(point, is_slow)
                    right_sample: Optional[Tuple[float, float, float]] = self._right.sample(point, is_slow)
                    if left_sample is None:
                        return right_sample
                    else:
                        if right_sample is None:
                            return left_sample
                        else:
                            # left and right samples are not None
                            # choose the closest to the inupt point
                            left_sq_distant = (left_sample[0] - point[0])**2 + (left_sample[1] - point[1])**2 + (left_sample[2] - point[2])**2
                            right_sq_distant = (right_sample[0] - point[0])**2 + (right_sample[1] - point[1])**2 + (right_sample[2] - point[2])**2

                            if left_sq_distant < right_sq_distant:
                                return left_sample
                            else:
                                return right_sample
                else:
                    if self._right is None:
                        # use left subtree
                        return self._left.sample(point)
                    else:
                        # use right subtree
                        return self._right.sample(point)
        else:
            return None


def polygons_to_triangles(vertices: List[Tuple[float, float, float]], polygons: List[List[int]]) -> List[Triangle]:
    '''Convart input arrays with polygonal description of the navmesh into array of triangles

    Input:
        vertices - array of vertex coordinates
        polygons - array of polygon indices

    Output:
        array of Triangle objects
    '''
    triangles = []
    for polygon in polygons:
        size = len(polygon)
        if size >= 3:
            for i in range(1, size - 1):
                triangles.append(Triangle([vertices[polygon[0]], vertices[polygon[i]], vertices[polygon[i + 1]]]))

    return triangles
