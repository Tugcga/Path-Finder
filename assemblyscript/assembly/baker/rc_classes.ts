import { RC_SPANS_PER_POOL } from "./rc_constants";
import { List } from "../common/list";

type int = i32;
type float = f64;

export class Span{
    smin: i32 = 0;  // 13 bits  // The lower limit of the span
    smax: i32 = 0;  // 13 bits  // The upper limit of the span
    area: i32 = 0;  // 6 bits  // The area id assigned to the span
    next: Span | null = null;  // The next span higher up in column

    to_string(): string{
        let n = this.next;
        if(n){
            return "<" + this.smin.toString() + ", " + this.smax.toString() + ", " + this.area.toString() + ", +" + ">";
        }
        else{
            return "<" + this.smin.toString() + ", " + this.smax.toString() + ", " + this.area.toString() + ", -" + ">";
        }
    }

    toString(): string{
        return this.to_string();
    }
}

export class Heightfield{
    width: i32 = 0;  // The width of the heightfield. (Along the x-axis in cell units.)
    height: i32 = 0;  // The height of the heightfield. (Along the z-axis in cell units.)
    bmin: StaticArray<float> = new StaticArray<float>(3);  // The minimum bounds in world space. [(x, y, z)]
    bmax: StaticArray<float> = new StaticArray<float>(3);  // The maximum bounds in world space. [(x, y, z)]
    cs: f64 = 0.0;  // The size of each cell. (On the xz-plane.)
    ch: f64 = 0.0;  // The height of each cell. (The minimum increment along the y-axis.)
    spans: StaticArray<Span|null> = new StaticArray<Span|null>(0);  // Heightfield of spans (width*height)
    freelist: Span | null = null;  // The next free span
}

export class CompactCell{
    index: int = 0;  // 24 bits  // Index to the first span in the column
    count: int = 0;  // 8 bits  // Number of spans in the column
}

export class CompactSpan{
    y: int = 0;  // 2 bytes  // The lower extent of the span (Measured from the heightfield's base)
    reg: int = 0;  // 2 bytes  // The id of the region the span belongs to (Or zero if not in a region)
    con: int = 0;  // Packed neighbor connection data
    h: int = 0;  // The height of the span (Measured from y)

    to_string(): string{
        return "<" + this.y.toString() + ", " + this.reg.toString() + ", " + this.con.toString() + ", " + this.h.toString() + ">";
    }
}

export class CompactHeightfield{
    width: int = 0;  // The width of the heightfield (Along the x-axis in cell units)
    height: int = 0;  // The height of the heightfield (Along the z-axis in cell units)
    span_count: int = 0;  // The number of spans in the heightfield
    walkable_height: int = 0;  // The walkable height used during the build of the field
    walkable_climb: int = 0;  // The walkable climb used during the build of the field
    border_size: int = 0;  // The AABB border size used during the build of the field
    max_distance: int = 0;  // 2 bytes  // The maximum distance value of any span within the field
    max_regions: int = 0;  // 2 bytes  // The maximum region id of any span within the field
    bmin: StaticArray<float> = new StaticArray<float>(3);
    bmax: StaticArray<float> = new StaticArray<float>(3);
    cs: float = 0.0;  // The size of each cell (On the xz-plane)
    ch: float = 0.0;  // The height of each cell (The minimum increment along the y-axis)
    cells: StaticArray<CompactCell | null> = new StaticArray<CompactCell | null>(0);  // Array of cells [Size: width*height]
    spans: StaticArray<CompactSpan | null> = new StaticArray<CompactSpan | null>(0);  // Array of spans [Size: spanCount]
    dist: StaticArray<int> = new StaticArray<int>(0);  // 2 bytes per element  // Array containing border distance data [Size: spanCount]
    areas: StaticArray<int> = new StaticArray<int>(0);  // 1 byte per element  // Array containing area id data [Size: spanCount]
}

export class LevelStackEntry{
    x: int = 0;
    y: int = 0;
    index: int = 0;

    constructor(_x: int = 0, _y: int = 0, _index: int = 0){
        this.x = _x;
        this.y = _y;
        this.index = _index;
    }

    toString(): string {
        return "LS[" + this.x.toString() + ", " + this.y.toString() + ", " + this.index.toString() + "]";
    }
}

export class DirtyEntry{
    index: int = 0;
    region: int = 0;
    distance2: int = 0;

    constructor(_index: int = 0,
                _region: int = 0,  // 2 bytes
                _distance2: int = 0){  // 2 bytes
        this.index = _index;
        this.region = _region;
        this.distance2 = _distance2;
    }
}

export class Region{
    span_count: int = 0;
    id: int = 0;  // 2 bytes
    area_type: int = 0;  // 1 byte
    remap: bool = false;
    visited: bool = false;
    overlap: bool = false;
    connects_to_border: bool = false;
    ymin: int = 0xffff;  // 65535  // 2 bytes
    ymax: int = 0;  // 2 bytes
    connections: List<int> = new List<int>();
    floors: List<int> = new List<int>();

    constructor(i: int){
        this.id = i;
    }

    to_string(): string{
        return "[" + this.id.toString() + ", " + this.span_count.toString() + ", " + this.overlap.toString() + "]";
    }
}

export class Contour{
    verts: StaticArray<int> = new StaticArray<int>(0);  // Simplified contour vertex and connection data [Size: 4 * nverts]
    nverts: int = 0;  // The number of vertices in the simplified contour
    rverts: StaticArray<int> = new StaticArray<int>(0);  // Raw contour vertex and connection data [Size: 4 * nrverts]
    nrverts: int = 0;  // The number of vertices in the raw contour
    reg: int = 0;  // 2 bytes  // The region id of the contour
    area: int = 0;  // 1 byte  // The area id of the contour

    toString(): string{
        return "Contour[" + this.nverts.toString() + " at " + this.reg.toString() + "]";
    }
}

export class ContourSet{
    conts: StaticArray<Contour> = new StaticArray<Contour>(0);  // An array of the contours in the set. [Size: nconts]
    nconts: int = 0;  // The number of contours in the set
    bmin: StaticArray<float> = new StaticArray<float>(3);
    bmax: StaticArray<float> = new StaticArray<float>(3);
    cs: float = 0.0;  // The size of each cell (On the xz-plane)
    ch: float = 0.0;  // The height of each cell (The minimum increment along the y-axis)
    width: int = 0;  // The width of the set (Along the x-axis in cell units) 
    height: int = 0;  // The height of the set (Along the z-axis in cell units) 
    border_size: int = 0;  // The AABB border size used to generate the source data from which the contours were derived
    max_error: float = 0.0;  // The max edge error that this contour set was simplified with
}

export class ContourHole{
    contour: Contour | null = null;
    minx: int = 0;
    minz: int = 0;
    leftmost: int = 0;
}

export class ContourRegion{
    outline: Contour | null = null;
    holes_index: int = 0;
    nholes: int = 0;
}

export class PotentialDiagonal{
    vert: int = 0;
    dist: int = 0;
}

export class PolyMesh{
    verts: StaticArray<int> = new StaticArray<int>(0);  // 2 bytes per element  // The mesh vertices [Form: (x, y, z) * nverts]
    polys: StaticArray<int> = new StaticArray<int>(0);  // 2 bytes  // Polygon and neighbor data [Length: maxpolys * 2 * nvp]
    regs: StaticArray<int> = new StaticArray<int>(0);  // 2 bytes  // The region id assigned to each polygon [Length: maxpolys]
    areas: StaticArray<int> = new StaticArray<int>(0);  // 1 byte  // The area id assigned to each polygon [Length: maxpolys]
    // all above are unsigned
    nverts: int = 0;  // The number of vertices
    npolys: int = 0;  // The number of polygons
    maxpolys: int = 0;  // The number of allocated polygons
    nvp: int = 0;  // The maximum number of vertices per polygon
    bmin: StaticArray<float> = new StaticArray<float>(3);
    bmax: StaticArray<float> = new StaticArray<float>(3);
    cs: float = 0.0;  // The size of each cell (On the xz-plane)
    ch: float = 0.0;  // The height of each cell (The minimum increment along the y-axis)
    border_size: int = 0;  // The AABB border size used to generate the source data from which the mesh was derived
    max_edge_error: float = 0.0;  // The max error of the polygon edges in the mesh
}

export class Edge{
    vert: StaticArray<int> = new StaticArray<int>(2);  // usigned 2 bytes, always two elements
    poly_edge: StaticArray<int> = new StaticArray<int>(2);
    poly: StaticArray<int> = new StaticArray<int>(2);
}
