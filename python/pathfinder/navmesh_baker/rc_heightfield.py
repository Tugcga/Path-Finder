from typing import List, Tuple, Optional
import math
from pathfinder.navmesh_baker.rc_classes import Span, Heightfield, CompactHeightfield, CompactCell, CompactSpan
from pathfinder.navmesh_baker.rc_constants import RC_PI, RC_WALKABLE_AREA, RC_NULL_AREA, RC_NOT_CONNECTED
from pathfinder.navmesh_baker.rc_calcs import calc_tri_normal, clamp, get_dir_offset_x, get_dir_offset_y, set_con

def create_height_field(width: int,
                        height: int,
                        bmin: Tuple[float, float, float],
                        bmax: Tuple[float, float, float],
                        cs: float,
                        ch: float) -> Heightfield:
    hf = Heightfield()
    hf.width = width
    hf.height = height
    hf.bmin = (bmin[0], bmin[1], bmin[2])
    hf.bmax = (bmax[0], bmax[1], bmax[2])
    hf.cs = cs
    hf.ch = ch
    hf.spans = [None for i in range(width * height)]

    return hf

# Only sets the area id's for the walkable triangles. Does not alter the area id's for unwalkable triangles
def mark_walkable_triangles(walkable_slope_angle: float,
                            verts,  # List of floats, but the size may change
                            nv: int,
                            tris,  # List of ints
                            nt: int):
    areas = [0] * nt

    walkable_thr = math.cos(RC_PI * walkable_slope_angle/180.0)
    norm = [0.0, 0.0, 0.0]  # buffer for normal vector
    for i in range(nt):
        tri_0, tri_1, tri_2 = tris[3*i : 3*i+3]
        calc_tri_normal(verts[3*tri_0 : 3*tri_0+3],  # may be this is not very good to create new lists for each vertex
                        verts[3*tri_1 : 3*tri_1+3], 
                        verts[3*tri_2 : 3*tri_2+3], norm)
        if norm[1] > walkable_thr:
            areas[i] = RC_WALKABLE_AREA

    return areas

def get_heightfield_span_count(hf: Heightfield) -> int:
    w: int = hf.width
    h: int = hf.height
    span_count: int = 0
    for y in range(h):
        for x in range(w):
            s: Optional[Span] = hf.spans[x + y*w]
            while s is not None:
                if s.area != RC_NULL_AREA:
                    span_count += 1
                s = s.next
    return span_count

def build_compact_heightfield(walkable_height: int,
                              walkable_climb: int,
                              hf: Heightfield) -> CompactHeightfield:
    chf = CompactHeightfield()
    w: int = hf.width
    h: int = hf.height
    span_count: int = get_heightfield_span_count(hf)
    chf.width = w
    chf.height = h
    chf.span_count = span_count
    chf.walkable_height = walkable_height
    chf.walkable_climb = walkable_climb
    chf.max_regions = 0
    chf.bmin = (hf.bmin[0], hf.bmin[1], hf.bmin[2])
    chf.bmax = (hf.bmax[0], hf.bmax[1] + walkable_height * hf.ch, hf.bmax[2])
    chf.cs = hf.cs
    chf.ch = hf.ch
    chf.cells = [None] * (w * h)  # type: ignore
    chf.spans = [None] * span_count
    chf.areas = [RC_NULL_AREA] * span_count
    chf.dist = [0] * span_count

    MAX_HEIGHT: int = 65535

    # Fill in cells and spans
    idx: int = 0
    for y in range(h):
        for x in range(w):
            s: Optional[Span] = hf.spans[x + y*w]
            c: CompactCell = CompactCell()
            chf.cells[x + y*w] = c
            # If there are no spans at this cell, just leave the data to index=0, count=0
            if s is None:
                continue
            c.index = idx
            c.count = 0
            while s is not None:
                if s.area != RC_NULL_AREA:
                    bot: int = s.smax
                    top: int = s.next.smin if s.next is not None else MAX_HEIGHT
                    if chf.spans[idx] is None:
                        chf.spans[idx] = CompactSpan()
                    cs: Optional[CompactSpan] = chf.spans[idx]
                    if cs is not None:
                        cs.y = clamp(bot, 0, 35535)
                        cs.h = clamp(top - bot, 0, 255)
                    chf.areas[idx] = s.area
                    idx += 1
                    c.count += 1
                s = s.next

    # Find neighbour connections
    MAX_LAYERS: int = RC_NOT_CONNECTED - 1
    too_high_neighbour: int = 0
    for y in range(h):
        for x in range(w):
            cc: Optional[CompactCell] = chf.cells[x + y*w]
            if cc is not None:
                for i in range(cc.index, cc.index + cc.count):
                    ss: Optional[CompactSpan] = chf.spans[i]
                    if ss is not None:
                        for dir in range(4):
                            set_con(ss, dir, RC_NOT_CONNECTED)
                            nx: int = x + get_dir_offset_x(dir)
                            ny: int = y + get_dir_offset_y(dir)
                            # First check that the neighbour cell is in bound
                            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                                continue

                            # Iterate over all neighbour spans and check if any of the is
                            # accessible from current cell
                            nc: Optional[CompactCell] = chf.cells[nx + ny*w]
                            if nc is not None:
                                for k in range(nc.index, nc.index + nc.count):
                                    ns: Optional[CompactSpan] = chf.spans[k]
                                    if ns is not None:
                                        bots: int = max(ss.y, ns.y)
                                        tops: int = min(ss.y + ss.h, ns.y + ns.h)

                                        # Check that the gap between the spans is walkable
                                        # and that the climb height between the gaps is not too high
                                        if tops - bots >= walkable_height and abs(ns.y - ss.y) <= walkable_climb:
                                            # Mark direction as walkable
                                            lidx: int = k - nc.index
                                            if lidx < 0 or lidx > MAX_LAYERS:
                                                too_high_neighbour = max(too_high_neighbour, lidx)
                                                continue
                                            set_con(ss, dir, lidx)
                                            break
    if too_high_neighbour > MAX_LAYERS:
        print("[Navmesh Baker] build_compact_heightfield: Heightfield has too many layers")

    return chf
