from typing import List, Tuple
import math
import struct
from pathfinder.navmesh_baker.rc_calcs import calc_grid_size
from pathfinder.navmesh_baker.rc_rasterization import rasterize_triangles
from pathfinder.navmesh_baker.rc_filtering import filter_low_hanging_walkable_obstacles, filter_ledge_spans, filter_walkable_low_height_spans
from pathfinder.navmesh_baker.rc_area import erode_walkable_area, build_distance_field, build_regions
from pathfinder.navmesh_baker.rc_contour import build_contours
from pathfinder.navmesh_baker.rc_classes import ContourSet, PolyMesh
from pathfinder.navmesh_baker.rc_mesh import build_poly_mesh
from pathfinder.navmesh_baker.rc_heightfield import create_height_field, mark_walkable_triangles, build_compact_heightfield

class NavmeshBaker:
    def __init__(self):
        self._input_vertices: List[Tuple[float, float, float]] = []  # list of vertex coordinates v0 = (x, y, z), v1 = (x, y, z), ...
        self._input_triangles: List[Tuple[int, int, int]] = []  # list of triange vertex indexes t0 = (i, j, k), t1 = (i, j, k), ...
        self._output_vertices: List[Tuple[float, float, float]] = []
        self._output_polygons: List[List[int]] = []
        self._is_dirty = True

    def add_geometry(self, vertices: List[Tuple[float, float, float]],
                           polygons: List[List[int]]):
        '''add geometry to the baker

            vertices - array of vertex positions, each position is 3-tuple
            polygons - array of arrays. Each array define polygon as a sequence of vertex indices
        '''
        n: int = len(self._input_vertices)  # remember the size of the initial array
        # add new vertices to the list
        self._input_vertices.extend(vertices)
        # add new triangle indices, but increase it values to n
        for polygon in polygons:
            # triangulate each polygon and add triangles
            for i in range(1, len(polygon) - 1):
                self._input_triangles.append((polygon[0] + n, polygon[i] + n, polygon[i + 1] + n))
        self._is_dirty = True

    def bake(self, cell_size: float = 0.3,
                   cell_height: float = 0.2,
                   agent_height: float = 2.0,
                   agent_radius: float = 0.6,
                   agent_max_climb: float = 0.9,
                   agent_max_slope: float = 45.0,
                   region_min_size: int = 8,
                   region_merge_size: int = 20,
                   edge_max_len: float = 12.0,
                   edge_max_error: float = 1.3,
                   verts_per_poly: int = 6,
                   detail_sample_distance: float = 6.0,
                   detail_sample_maximum_error: float = 1.0) -> bool:
        nverts = len(self._input_vertices)
        ntris = len(self._input_triangles)
        # convert input vertices and triangles to plain lists
        verts = [x for t in self._input_vertices for x in t]
        tris = [x for t in self._input_triangles for x in t]
        self._output_vertices = []
        self._output_polygons = []
        if nverts > 0 and ntris > 0:
            # Step 1. Initialize build config
            # calculate input geometry bounding box
            infinity = float("inf")
            bb_min: List[float] = [infinity, infinity, infinity]
            bb_max: List[float] = [-infinity, -infinity, -infinity]
            for v in self._input_vertices:
                x, y, z = v
                if x < bb_min[0]:
                    bb_min[0] = x
                if x > bb_max[0]:
                    bb_max[0] = x
                if y < bb_min[1]:
                    bb_min[1] = y
                if y > bb_max[1]:
                    bb_max[1] = y
                if z < bb_min[2]:
                    bb_min[2] = z
                if z > bb_max[2]:
                    bb_max[2] = z
            bmin = (bb_min[0], bb_min[1], bb_min[2])
            bmax = (bb_max[0], bb_max[1], bb_max[2])
            
            # setup bake parameters
            cs = cell_size
            ch = cell_height
            walkable_slope_angle = agent_max_slope
            walkable_height = math.ceil(agent_height / ch)
            walkable_climb = math.floor(agent_max_climb / ch)
            walkable_radius = math.ceil(agent_radius / cs)
            max_edge_len = int(edge_max_len / cs)
            max_simplification_error = edge_max_error
            min_region_area = region_min_size**2
            merge_region_area = region_merge_size**2
            max_verts_per_poly = verts_per_poly
            detail_sample_dist = 0.0 if detail_sample_distance < 0.9 else cs * detail_sample_distance
            detail_sample_max_error = ch * detail_sample_maximum_error

            # Set the area where the navigation will be build
            width, height = calc_grid_size(bmin, bmax, cs)

            # Step 2. Rasterize input polygon soup
            solid = create_height_field(width, height, bmin, bmax, cs, ch)

            # Find triangles which are walkable based on their slope and rasterize them
            triareas = mark_walkable_triangles(walkable_slope_angle, verts, nverts, tris, ntris)
            is_rasterize = rasterize_triangles(verts, tris, triareas, ntris, solid, walkable_climb)
            if not is_rasterize:
                print("[Navmesh Baker] bake: Could not rasterize triangles")
                return False

            # Step 3. Filter walkables surfaces
            # Once all geoemtry is rasterized, we do initial pass of filtering to
            # remove unwanted overhangs caused by the conservative rasterization
            # as well as filter spans where the character cannot possibly stand
            filter_low_hanging_walkable_obstacles(walkable_climb, solid)
            filter_ledge_spans(walkable_height, walkable_climb, solid)
            filter_walkable_low_height_spans(walkable_height, solid)

            # Step 4. Partition walkable surface to simple regions
            # Compact the heightfield so that it is faster to handle from now on
            # This will result more cache coherent data as well as the neighbours
            # between walkable cells will be calculated
            chf = build_compact_heightfield(walkable_height, walkable_climb, solid)
            # Erode the walkable area by agent radius
            is_erode: bool = erode_walkable_area(walkable_radius, chf)
            if not is_erode:
                print("[Navmesh Baker] bake: Could not erode")
                return False

            # Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas
            is_build_field: bool = build_distance_field(chf)
            if not is_build_field:
                print("[Navmesh Baker] bake: Could not build distance field")
                return False

            is_build_regions: bool = build_regions(chf, 0, min_region_area, merge_region_area)
            if not is_build_regions:
                print("[Navmesh Baker] bake: Could not build watershed regions")
                return False

            # Step 5. Trace and simplify region contours
            # Create contours
            cset = ContourSet()
            is_build_contours = build_contours(chf, max_simplification_error, max_edge_len, cset)
            if not is_build_contours:
                print("[Navmesh Baker] bake: Could not create contours")
                return False

            # Step 6. Build polygons mesh from contours
            pmesh: PolyMesh = PolyMesh()
            is_build_polymesh = build_poly_mesh(cset, max_verts_per_poly, pmesh)
            if not is_build_polymesh:
                print("[Navmesh Baker] bake: Could not triangulate contours")

            # generate output mesh data
            # vertices
            for v_index in range(pmesh.nverts):
                self._output_vertices.append((pmesh.bmin[0] + pmesh.cs * pmesh.verts[3*v_index], pmesh.bmin[1] + pmesh.ch * (pmesh.verts[3*v_index + 1] - 1), pmesh.bmin[2] + pmesh.cs * pmesh.verts[3*v_index + 2]))
            # polygons
            polygon: List[int] = []
            for p_index in range(pmesh.npolys):
                pv: int = p_index * 2 * pmesh.nvp
                for j in range(pmesh.nvp):
                    vv = pmesh.polys[pv + j]
                    if vv == 0xffff:
                        break
                    polygon.append(pmesh.polys[pv + j])
                self._output_polygons.append(polygon)
                polygon = []

            self._is_dirty = False

            return True
        else:
            print("[Navmesh Baker] bake: Input geometry is empty, stop baking process")
            return False

    def get_polygonization(self) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
        '''return a tubple (vertices, polygons)

        vertices is array of 3-tuples [(x1, y1, z1), (x2, y2, z2), ...]
        polygons is array of arrays [[p11, p12, ...], [p21, p22, ...], ...]

        if navmesh is not constructed, return emoty tuple
        '''
        if self._is_dirty:
            print("[Navmesh Baker] get_polygonization: Navigation mesh is not constructed. Call bake() at first")
            return ([], [])
        else:
            return (self._output_vertices, self._output_polygons)

    def save_to_binary(self, file_path: str):
        '''Save baked navmesh to the binary file

        The structure of the file is very simple
        It starts from sequence of 32 bit floats, which corresponds to vertex positions
        Infinite float define the end of the array
        Then the file contains array of int, which define polygon vertex indices
        The also infine float as array end
        The last array is arrays of integers with polygon sizes
        THe file ends by inifine 32 bits float

        Byte order is big-endian
        '''
        if len(self._output_vertices) > 0 and len(self._output_polygons) > 0:
            vertices: List[float] = [x for t in self._output_vertices for x in t]
            polygons: List[int] = [x for t in self._output_polygons for x in t]
            sizes: List[int] = [len(polygon) for polygon in self._output_polygons]
            out_bytes = bytearray()
            for v in vertices:
                out_bytes.extend(struct.pack(">f", v))
            inf_float_bytes = struct.pack(">f", float("inf"))
            out_bytes.extend(inf_float_bytes)
            for p in polygons:
                out_bytes.extend(struct.pack(">i", p))
            out_bytes.extend(inf_float_bytes)
            for s in sizes:
                out_bytes.extend(struct.pack(">i", s))
            out_bytes.extend(inf_float_bytes)
            with open(file_path, "wb") as file:
                file.write(out_bytes)
        else:
            print("[Navmesh Baker] save_to_binary: Bake navigation mesh at first")

    def save_to_text(self, file_path: str):
        '''Save baked navigation mesh into text file

        The structure of the file is the following
        The first line is a plane sequance of floats with vertex coordinates, splitted by spaces
        The second line contains indexes of polygon vertices
        The last line contains sizes of polygons
        '''
        if len(self._output_vertices) > 0 and len(self._output_polygons) > 0:
            vertices: List[str] = [str(x) for t in self._output_vertices for x in t]
            polygons: List[str] = [str(x) for t in self._output_polygons for x in t]
            sizes: List[str] = [str(len(polygon)) for polygon in self._output_polygons]
            with open(file_path, "w") as file:
                file.write(" ".join(vertices))
                file.write("\n")
                file.write(" ".join(polygons))
                file.write("\n")
                file.write(" ".join(sizes))
        else:
            print("[Navmesh Baker] save_to_text: Bake navigation mesh at first")
