import unittest

from pathfinder.navmesh.navmesh_triangle import Triangle, TrianglesBVH, polygons_to_triangles, cross, dot
from pathfinder.navmesh import Navmesh
from pathfinder.navmesh.navmesh_graph import NavmeshGraph


class TestTriangle(unittest.TestCase):
    def test_triangle_closest_point_01(self):
        t = Triangle([(0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)])
        p0 = (0.25, 0.0, 0.25)
        p1 = (1.0, 0.0, 1.0)
        p2 = (2.0, 0.0, -1.0)
        p3 = (0.5, 0.0, -1.0)
        p4 = (-1.0, 0.0, -1.0)
        p5 = (-1.0, 0.0, 0.5)
        p6 = (-1.0, 0.0, 2.0)

        self.assertEqual(t.get_closest_point(p0), (0.25, 0.0, 0.25))
        self.assertEqual(t.get_closest_point(p1), (0.5, 0.0, 0.5))
        self.assertEqual(t.get_closest_point(p2), (1.0, 0.0, 0.0))
        self.assertEqual(t.get_closest_point(p3), (0.5, 0.0, 0.0))
        self.assertEqual(t.get_closest_point(p4), (0.0, 0.0, 0.0))
        self.assertEqual(t.get_closest_point(p5), (0.0, 0.0, 0.5))
        self.assertEqual(t.get_closest_point(p6), (0.0, 0.0, 1.0))

    def test_triangle_closest_point_02(self):
        t = Triangle([(1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0)])
        self.assertEqual(t.get_closest_point((1.5, 0.0, 0.5)), (1.0, 0.0, 0.5))

    def test_triangle_aabb(self):
        t = Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (2.0, 0.0, 0.0)])
        self.assertEqual(t.get_aabb(), (0.0, 0.0, 0.0, 2.0, 0.0, 1.0))


class TestTriangleBVH(unittest.TestCase):
    def test_sample_01(self):
        triangles = [Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)]),
                     Triangle([(1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0)])]
        tree = TrianglesBVH(triangles)
        self.assertEqual(tree.sample((1.5, 0.0, 0.5), is_slow=True), (1.0, 0.0, 0.5))
        self.assertEqual(tree.sample((1.5, 0.0, 0.5), is_slow=False), None)

    def test_sample_02(self):
        triangles = [Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)]),
                     Triangle([(1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0)])]
        tree = TrianglesBVH(triangles, aabb_delta=1.0)
        self.assertEqual(tree.sample((1.5, 0.0, 0.5), is_slow=False), (1.0, 0.0, 0.5))


class TestBuildTriangles(unittest.TestCase):
    def test_build_triangles_01(self):
        vertices = [(-3.0, 0.0, -3.0), (3.0, 0.0, -3.0), (3.0, 0.0, 3.0), (-3.0, 0.0, 3.0), (-1.0, 0.0, -1.0), (1.0, 0.0, -1.0), (1.0, 0.0, 1.0), (-1.0, 0.0, 1.0)]
        polygons = [[0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]

        triangles = polygons_to_triangles(vertices, polygons)
        self.assertEqual(str(triangles), "[[(-3.0, 0.0, -3.0), (-1.0, 0.0, -1.0), (1.0, 0.0, -1.0)], [(-3.0, 0.0, -3.0), (1.0, 0.0, -1.0), (3.0, 0.0, -3.0)], [(3.0, 0.0, -3.0), (1.0, 0.0, -1.0), (1.0, 0.0, 1.0)], [(3.0, 0.0, -3.0), (1.0, 0.0, 1.0), (3.0, 0.0, 3.0)], [(3.0, 0.0, 3.0), (1.0, 0.0, 1.0), (-1.0, 0.0, 1.0)], [(3.0, 0.0, 3.0), (-1.0, 0.0, 1.0), (-3.0, 0.0, 3.0)], [(-3.0, 0.0, 3.0), (-1.0, 0.0, 1.0), (-1.0, 0.0, -1.0)], [(-3.0, 0.0, 3.0), (-1.0, 0.0, -1.0), (-3.0, 0.0, -3.0)]]")


class TestSampleNavmesh(unittest.TestCase):
    def test_smaple_navmesh(self):
        vertices = [(-3.0, 0.0, -3.0), (3.0, 0.0, -3.0), (3.0, 0.0, 3.0), (-3.0, 0.0, 3.0), (-1.0, 0.0, -1.0), (1.0, 0.0, -1.0), (1.0, 0.0, 1.0), (-1.0, 0.0, 1.0)]
        polygons = [[0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]
        navmesh = Navmesh(vertices, polygons)
        p0 = navmesh.sample((1.5, 0.0, 1.5))
        self.assertEqual(p0, (1.5, 0.0, 1.5))

        p1 = navmesh.sample((0.0, 0.0, 0.0))
        self.assertEqual(p1, None)


class TestCross(unittest.TestCase):
    def test_cross_01(self):
        a = (1.0, 0.0, 0.0)
        b = (0.0, 1.0, 0.0)
        c = (0.0, 0.0, 1.0)
        self.assertEqual(cross(a, b), c)

    def test_cross_02(self):
        a = (1.0, 2.0, 3.0)
        b = (-1.0, 1.0, 2.0)
        c = (1.0, -5.0, 3.0)
        self.assertEqual(cross(a, b), c)


class TestDot(unittest.TestCase):
    def test_dot_01(self):
        a = (1.0, 0.0, 0.0)
        b = (0.0, 1.0, 0.0)
        self.assertEqual(dot(a, b), 0.0)

    def test_dot_02(self):
        a = (1.0, 1.0, 2.0)
        b = (2.0, 3.0, -1.0)
        self.assertEqual(dot(a, b), 3.0)


class TestTriangleRaycast(unittest.TestCase):
    def test_raycast_01(self):
        t = Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
        direction = (0.0, 0.0, -1.0)
        origin = (0.0, 0.0, 1.0)
        self.assertEqual(t.raycast(origin, direction), (0.0, 0.0, 0.0))

    def test_raycast_02(self):
        t = Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
        direction = (0.0, 0.0, -1.0)
        origin = (1.0, 1.0, 1.0)
        self.assertEqual(t.raycast(origin, direction), None)

    def test_raycast_03(self):
        t = Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
        direction = (0.0, 0.0, 1.0)
        origin = (0.25, 0.25, -1.0)
        self.assertEqual(t.raycast(origin, direction), (0.25, 0.25, 0.0))

    def test_raycast_04(self):
        t = Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
        direction = (0.0, 0.0, 1.0)
        origin = (0.25, 0.25, 1.0)
        self.assertEqual(t.raycast(origin, direction), None)

    def test_raycast_05(self):
        t = Triangle([(2.0, 0.0, 0.0), (0.0, 2.0, 0.0), (1.0, 1.0, 4.0)])
        direction = (1.0, 1.0, 0.0)
        origin = (-1.0, -1.0, 1.0)
        self.assertEqual(t.raycast(origin, direction), (1.0, 1.0, 1.0))

    def test_raycast_06(self):
        t = Triangle([(2.0, 0.0, 0.0), (0.0, 2.0, 0.0), (1.0, 1.0, 4.0)])
        direction = (1.0, 5.0, 0.0)
        origin = (-1.0, -1.0, 1.0)
        self.assertEqual(t.raycast(origin, direction), None)


class TestTrianglesBVHRaycast(unittest.TestCase):
    def test_triangles_bvh_01(self):
        triangles = [Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)]),
                     Triangle([(1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0)])]
        tree = TrianglesBVH(triangles)
        origin = (0.5, 1.0, 0.5)
        direction = (0.0, -1.0, 0.0)
        self.assertEqual(tree.raycast(origin, direction), (0.5, 0.0, 0.5))

    def test_triangles_bvh_02(self):
        triangles = [Triangle([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)]),
                     Triangle([(1.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0)])]
        tree = TrianglesBVH(triangles)
        origin = (-0.5, 1.0, 0.5)
        direction = (0.0, -1.0, 0.0)
        self.assertEqual(tree.raycast(origin, direction), None)


class TestGraph(unittest.TestCase):
    def test_search_01(self):
        vertices = [(1.0, 0.0, 1.0), (3.0, 0.0, 2.0), (2.0, 0.0, 4.0), (2.0, 0.0, -1.0)]
        names = [1, 2, 3, 4]
        edges = [(1, 3), (2, 3), (1, 4), (2, 4)]
        graph = NavmeshGraph(vertices, names, edges)
        path = graph.search(1, 2)
        self.assertEqual(path, [1, 3, 2])

    def test_search_02(self):
        vertices = [
            (2.0, 0.0, 2.0),
            (1.0, 0.0, 3.0),
            (3.0, 0.0, 3.0),
            (3.0, 0.0, 1.0),
            (1.0, 0.0, 1.0),
            (1.0, 0.0, 4.0),
            (0.0, 0.0, 3.0),
            (3.0, 0.0, 4.0),
            (4.0, 0.0, 3.0),
            (4.0, 0.0, 1.0),
            (3.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0)]
        names = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 5), (1, 6), (2, 7), (2, 8),
                 (3, 9), (3, 10), (4, 11), (4, 12), (1, 2), (2, 3), (3, 4), (1, 4),
                 (5, 6), (5, 7), (7, 8), (8, 9), (9, 10), (10, 11), (11, 12), (12, 6)]
        graph = NavmeshGraph(vertices, names, edges)
        self.assertEqual(graph.search(12, 9), [12, 4, 3, 9])
        self.assertEqual(graph.search(11, 7), [11, 4, 0, 2, 7])
        self.assertEqual(graph.search(3, 6), [3, 0, 1, 6])

    def test_search_03(self):
        vertices = [
            (-2.0, 0.0, 0.0),
            (-1.0, 0.0, -1.0),
            (-1.0, 0.0, 1.0),
            (1.0, 0.0, -1.0),
            (1.0, 0.0, 1.0),
            (2.0, 0.0, 0.0)]
        names = [1, 5, 0, 2, 4, 3]
        edges = [(1, 0), (1, 5), (0, 4), (5, 2), (4, 3), (2, 3), (0, 2), (5, 4)]
        graph = NavmeshGraph(vertices, names, edges)
        self.assertEqual(graph.search(1, 3), [1, 0, 4, 3])
        self.assertEqual(graph.search(5, 3), [5, 2, 3])

    def test_collect_01(self):
        vertices = [(-2.0, 0.0, 0.0),
                    (-1.0, 0.0, -1.0),
                    (-1.0, 0.0, 1.0),
                    (1.0, 0.0, -1.0),
                    (1.0, 0.0, 1.0),
                    (2.0, 0.0, 0.0)]
        names = [1, 5, 0, 2, 4, 3]
        edges = [(1, 0), (1, 5), (0, 4), (5, 2), (4, 3), (2, 3), (0, 2), (5, 4)]
        graph = NavmeshGraph(vertices, names, edges)
        min_path = graph.search(1, 3)
        self.assertEqual(min_path, [1, 0, 4, 3])
        self.assertEqual(graph.collect_pathes(min_path, 1.0), [[1, 0, 4, 3], [1, 5, 2, 3]])
        self.assertEqual(graph.collect_pathes(min_path, 1.5), [[1, 0, 4, 3], [1, 0, 2, 3], [1, 5, 2, 3], [1, 5, 4, 3]])

    def test_collet_02(self):
        vertices = [(-0.025000000000000022, 0.0, -2.275), (-2.2750000000000004, 0.0, 0.12499999999999994), (2.375, 0.0, -0.025000000000000022), (0.12499999999999994, 0.0, 2.3750000000000004)]
        names = [0, 1, 2, 3]
        edges = [(0, 1), (0, 2), (1, 3), (2, 3)]
        graph = NavmeshGraph(vertices, names, edges)
        min_path = graph.search(0, 3)
        self.assertEqual(min_path, [0, 2, 3])
        self.assertEqual(graph.collect_pathes(min_path, 1.1), [[0, 1, 3], [0, 2, 3]])


class TestNavmesh(unittest.TestCase):
    def test_search_path(self):
        vertices = [(-3.1, 0.0, -3.1), (-3.1, 0.0, 3.2), (3.2, 0.0, 3.2), (3.2, 0.0, -3.1), (-1.6, 0.0, -1.3), (1.4, 0.0, -1.6), (1.7, 0.0, 1.4), (-1.3, 0.0, 1.7)]
        polygons = [[0, 4, 5, 3], [4, 0, 1, 7], [3, 5, 6, 2], [7, 1, 2, 6]]
        navmesh = Navmesh(vertices, polygons)
        self.assertEqual(navmesh.search_path((0.0, 0.0, -2.0), (0.0, 0.0, 2.0)), [(0.0, 0.0, -2.0), (1.4, 0.0, -1.6), (1.7, 0.0, 1.4), (0.0, 0.0, 2.0)])
        self.assertEqual(navmesh.search_path((-2.0, 0.0, -2.5), (-2.0, 0.0, 2.5), 1.1), [(-2.0, 0.0, -2.5), (-2.0, 0.0, 2.5)])
        self.assertEqual(navmesh.search_path((-2.0, 0.0, -2.5), (-2.0, 0.0, 2.5)), [(-2.0, 0.0, -2.5), (1.4, 0.0, -1.6), (1.7, 0.0, 1.4), (-2.0, 0.0, 2.5)])


if __name__ == "__main__":
    unittest.main()
