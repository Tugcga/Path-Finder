import unittest

from pathfinder.navmesh.navmesh_triangle import Triangle, TrianglesBVH, polygons_to_triangles
from pathfinder.navmesh import Navmesh


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


if __name__ == "__main__":
    unittest.main()
