from typing import Optional, List, Tuple
from pathfinder.navmesh_baker.rc_constants import RC_SPANS_PER_POOL

# Represents a span in a heightfield
class Span:
    def __init__(self):
        self.smin = 0  # 13 bits  # The lower limit of the span
        self.smax = 0  # 13 bits  # The upper limit of the span
        self.area = 0  # 6 bits  # The area id assigned to the span
        self.next: Optional[Span] = None  # The next span higher up in column

    def __repr__(self) -> str:
        return "<" + str(self.smin) + ", " + str(self.smax) + ", " + str(self.area) + ", " + ("-" if self.next is None else "+") + ">"

# A dynamic heightfield representing obstructed space
class Heightfield:
    def __init__(self):
        self.width = 0  # The width of the heightfield. (Along the x-axis in cell units.)
        self.height = 0  # The height of the heightfield. (Along the z-axis in cell units.)
        self.bmin = (0.0, 0.0, 0.0)  # The minimum bounds in world space. [(x, y, z)]
        self.bmax = (0.0, 0.0, 0.0)  # The maximum bounds in world space. [(x, y, z)]
        self.cs = 0.0  # The size of each cell. (On the xz-plane.)
        self.ch = 0.0  # The height of each cell. (The minimum increment along the y-axis.)
        self.spans: List[Optional[Span]] = []  # Heightfield of spans (width*height)
        self.freelist: Optional[Span] = None  # The next free span

# Provides information on the content of a cell column in a compact heightfield
class CompactCell:
    def __init__(self):
        self.index: int = 0  # 24 bits  # Index to the first span in the column
        self.count: int = 0  # 8 bits  # Number of spans in the column

class CompactSpan:
    def __init__(self):
        self.y: int = 0  # 2 bytes  # The lower extent of the span (Measured from the heightfield's base)
        self.reg: int = 0  # 2 bytes  # The id of the region the span belongs to (Or zero if not in a region)
        self.con: int = 0  # Packed neighbor connection data
        self.h: int = 0  # The height of the span (Measured from y)

    def __repr__(self) -> str:
        return "<" + str(self.y) + ", " + str(self.reg) + ", " + str(self.con) + ", " + str(self.h) + ">"

# A compact, static heightfield representing unobstructed space
class CompactHeightfield:
    def __init__(self):
        self.width: int = 0  # The width of the heightfield (Along the x-axis in cell units)
        self.height: int = 0  # The height of the heightfield (Along the z-axis in cell units)
        self.span_count: int = 0  # The number of spans in the heightfield
        self.walkable_height: int = 0  # The walkable height used during the build of the field
        self.walkable_climb: int = 0  # The walkable climb used during the build of the field
        self.border_size: int = 0  # The AABB border size used during the build of the field
        self.max_distance: int = 0  # 2 bytes  # The maximum distance value of any span within the field
        self.max_regions: int = 0  # 2 bytes  # The maximum region id of any span within the field
        self.bmin: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # The minimum bounds in world space [(x, y, z)]
        self.bmax: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # The maximum bounds in world space [(x, y, z)]
        self.cs: float = 0.0  # The size of each cell (On the xz-plane)
        self.ch: float = 0.0  # The height of each cell (The minimum increment along the y-axis)
        self.cells: List[Optional[CompactCell]] = []  # Array of cells [Size: width*height]
        self.spans: List[Optional[CompactSpan]] = []  # Array of spans [Size: spanCount]
        self.dist: List[int] = []  # 2 bytes per element  # Array containing border distance data [Size: spanCount]
        self.areas: List[int] = []  # 1 byte per element  # Array containing area id data [Size: spanCount]

class LevelStackEntry:
    def __init__(self, _x: int = 0, _y: int = 0, _index: int = 0):
        self.x = _x
        self.y = _y
        self.index = _index

    def __repr__(self):
        return "LS[" + str(self.x) + ", " + str(self.y) + ", " + str(self.index) + "]"

class DirtyEntry:
    def __init__(self, _index: int = 0,
                       _region: int = 0,  # 2 bytes
                       _distance2: int = 0):  # 2 bytes
        self.index = _index
        self.region = _region
        self.distance2 = _distance2

class Region:
    def __init__(self, i: int = 0):
        self.span_count: int = 0
        self.id: int = i  # 2 bytes
        self.area_type: int = 0  # 1 byte
        self.remap: bool = False
        self.visited: bool = False
        self.overlap: bool = False
        self.connects_to_border: bool = False
        self.ymin: int = 0xffff  # 65535  # 2 bytes
        self.ymax: int = 0  # 2 bytes
        self.connections: List[int] = []
        self.floors: List[int] = []

    def __repr__(self) -> str:
        return "[" + str(self.id) + ", " + str(self.span_count) + ", " + str(self.overlap) + "]"

class Contour:
    def __init__(self):
        self.verts: List[int] = []  # Simplified contour vertex and connection data [Size: 4 * nverts]
        self.nverts: int = 0  # The number of vertices in the simplified contour
        self.rverts: List[int] = []  # Raw contour vertex and connection data [Size: 4 * nrverts]
        self.nrverts: int = 0  # The number of vertices in the raw contour
        self.reg: int = 0  # 2 bytes  # The region id of the contour
        self.area: int = 0  # 1 byte  # The area id of the contour

class ContourSet:
    def __init__(self):
        self.conts: List[Contour] = []  # An array of the contours in the set. [Size: nconts]
        self.nconts: int = 0  # The number of contours in the set
        self.bmin: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # The minimum bounds in world space [(x, y, z)]
        self.bmax: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # The maximum bounds in world space [(x, y, z)]
        self.cs: float = 0.0  # The size of each cell (On the xz-plane)
        self.ch: float = 0.0  # The height of each cell (The minimum increment along the y-axis)
        self.width: int = 0  # The width of the set (Along the x-axis in cell units) 
        self.height: int = 0  # The height of the set (Along the z-axis in cell units) 
        self.border_size: int = 0  # The AABB border size used to generate the source data from which the contours were derived
        self.max_error: float = 0.0  # The max edge error that this contour set was simplified with

class ContourHole:
    def __init__(self):
        self.contour: Optional[Contour] = None
        self.minx: int = 0
        self.minz: int = 0
        self.leftmost: int = 0

class ContourRegion:
    def __init__(self):
        self.outline: Optional[Contour] = None
        self.holes_index: int = 0
        self.nholes: int = 0

class PotentialDiagonal:
    def __init__(self):
        self.vert: int = 0
        self.dist: int = 0

class PolyMesh:
    def __init__(self):
        self.verts: List[int] = []  # 2 bytes per element  # The mesh vertices [Form: (x, y, z) * nverts]
        self.polys: List[int] = []  # 2 bytes  # Polygon and neighbor data [Length: maxpolys * 2 * nvp]
        self.regs: List[int] = []  # 2 bytes  # The region id assigned to each polygon [Length: maxpolys]
        self.flags: List[int] = []  # 2 bytes  # The user defined flags for each polygon [Length: maxpolys]
        self.areas: List[int] = []  # 1 byte  # The area id assigned to each polygon [Length: maxpolys]
        # all above are unsigned
        self.nverts: int = 0  # The number of vertices
        self.npolys: int = 0  # The number of polygons
        self.maxpolys: int = 0  # The number of allocated polygons
        self.nvp: int = 0  # The maximum number of vertices per polygon
        self.bmin: Typle[float, float, float] = (0.0, 0.0, 0.0)  # The minimum bounds in world space [(x, y, z)]
        self.bmax: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # The maximum bounds in world space [(x, y, z)]
        self.cs: float = 0.0  # The size of each cell (On the xz-plane)
        self.ch: float = 0.0  # The height of each cell (The minimum increment along the y-axis)
        self.border_size: int = 0  # The AABB border size used to generate the source data from which the mesh was derived
        self.max_edge_error: float = 0.0  # The max error of the polygon edges in the mesh

class Edge:
    def __init__(self):
        self.vert: List[int] = [0, 0]  # usigned 2 bytes, always two elements
        self.poly_edge: List[int] = [0, 0]
        self.poly: List[int] = [0, 0]
