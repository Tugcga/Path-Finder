from typing import List, Tuple
import math
from pathfinder.navmesh_baker.rc_calcs import v_copy, v_min, v_max, overlap_bounds, clamp
from pathfinder.navmesh_baker.rc_classes import Span, Heightfield
from pathfinder.navmesh_baker.rc_constants import RC_SPAN_MAX_HEIGHT

def divide_poly(buf: List[float],
                in_ptr: int,
                nin: int,
                out1: int, out2: int,  # pointers in the buffer where we should write the data
                x: float,
                axis: int) -> Tuple[int, int]:  # return nout1 and nout2
    d: List[float] = [0.0] * 12
    for i in range(nin):
        d[i] = x - buf[3*i + axis + in_ptr]
    m: int = 0
    n: int = 0

    j: int = nin - 1
    for i in range(nin):
        ina: bool = d[j] >= 0.0
        inb: bool = d[i] >= 0.0
        if ina != inb:
            s: float = d[j] / (d[j] - d[i])
            buf[m*3 + out1] = buf[j*3 + in_ptr] + (buf[i*3 + in_ptr] - buf[j*3 + in_ptr]) * s
            buf[m*3 + out1 + 1] = buf[j*3 + 1 + in_ptr] + (buf[i*3 + 1 + in_ptr] - buf[j*3 + 1 + in_ptr]) * s
            buf[m*3 + out1 + 2] = buf[j*3 + 2 + in_ptr] + (buf[i*3 + 2 + in_ptr] - buf[j*3 + 2 + in_ptr]) * s
            v_copy(buf, buf, dest_shift=out2+n*3, src_shift=out1+m*3)
            m += 1
            n += 1
            # add the i'th point to the right polygon. Do NOT add points that are on the dividing line
            # since these were already added above
            if d[i] > 0:
                v_copy(buf, buf, dest_shift=out1+m*3, src_shift=in_ptr+i*3)
                m += 1
            elif d[i] < 0:
                v_copy(buf, buf, dest_shift=out2+n*3, src_shift=in_ptr+i*3)
                n += 1
        else:  # same side
            # add the i'th point to the right polygon. Addition is done even for points on the dividing line
            if d[i] >= 0:
                v_copy(buf, buf, dest_shift=out1+m*3, src_shift=in_ptr+i*3)
                m += 1
                if d[i] != 0.0:
                    j = i
                    continue
            v_copy(buf, buf, dest_shift=out2+n*3, src_shift=in_ptr+i*3)
            n += 1

        j = i

    return m, n


def free_span(hf: Heightfield, ptr: Span):
    ptr.next = hf.freelist
    hf.freelist = ptr


def add_span(hf: Heightfield,  # modify this object
             x: int,
             y: int,
             smin: int,
             smax: int,
             area: int, 
             flag_mege_thr: int) -> bool:
    idx = x + y * hf.width
    s = Span()
    s.smin = smin
    s.smax = smax
    s.area = area
    s.next = None

    # Empty cell, add the first span
    if hf.spans[idx] is None:
        hf.spans[idx] = s
        return True
    prev = None
    cur = hf.spans[idx]

    # Insert and merge spans
    while cur is not None:
        if cur.smin > s.smax:
            # Current span is further than the new span, break
            break
        elif cur.smax < s.smin:
            #  Current span is before the new span advance
            prev = cur
            cur = cur.next
        else:
            # Merge spans
            if cur.smin < s.smin:
                s.smin = cur.smin
            if cur.smax > s.smax:
                s.smax = cur.smax

            # Merge flags
            if abs(s.smax - cur.smax) <= flag_mege_thr:
                s.area = max(s.area, cur.area)

            # Remove current span
            next_span = cur.next
            free_span(hf, cur)
            if prev is not None:
                prev.next = next_span
            else:
                hf.spans[idx] = next_span
            cur = next_span
    # Insert new span
    if prev is not None:
        s.next = prev.next
        prev.next = s
    else:
        s.next = hf.spans[idx]
        hf.spans[idx] = s

    return True

def rasterize_tri(v0: List[float],
                  v1: List[float],
                  v2: List[float],
                  area: int,
                  hf: Heightfield,  # modify this object
                  bmin: Tuple[float, float, float],
                  bmax: Tuple[float, float, float],
                  cs: float,
                  ics: float,
                  ich: float,
                  flag_merge_thr: int) -> bool:
    w: int = hf.width
    h: int = hf.height
    t_min = [0.0, 0.0, 0.0]
    t_max = [0.0, 0.0, 0.0]
    by: float = bmax[1] - bmin[1]

    # Calculate the bounding box of the triangle
    v_copy(t_min, v0)
    v_copy(t_max, v0)
    v_min(t_min, v1)
    v_min(t_min, v2)
    v_max(t_max, v1)
    v_max(t_max, v2)

    # If the triangle does not touch the bbox of the heightfield, skip the triagle
    if not overlap_bounds(bmin, bmax, t_min, t_max):
        return True

    # Calculate the footprint of the triangle on the grid's y-axis
    y0: int = int((t_min[2] - bmin[2]) * ics)
    y1: int = int((t_max[2] - bmin[2]) * ics)
    y0 = clamp(y0, 0, h - 1)
    y1 = clamp(y1, 0, h - 1)

    # Clip the triangle into all grid cells it touches
    buf: List[float] = [0.0] * (7 * 3 * 4)
    in_ptr: int = 0
    inrow_ptr: int = 7*3
    p1_ptr: int = inrow_ptr + 7*3
    p2_ptr: int = p1_ptr + 7*3
    v_copy(buf, v0)
    v_copy(buf, v1, dest_shift=3)
    v_copy(buf, v2, dest_shift=6)
    nvrow: int = 3
    nv_in: int = 3

    for y in range(y0, y1 + 1):
        # Clip polygon to row. Store the remaining polygon as well
        cz: float = bmin[2] + y * cs
        nvrow, nv_in = divide_poly(buf, in_ptr, nv_in, inrow_ptr, p1_ptr, cz+cs, 2)
        in_ptr, p1_ptr = p1_ptr, in_ptr
        if nvrow < 3:
            continue
        # find the horizontal bounds in the row
        min_x = buf[inrow_ptr]
        max_x = buf[inrow_ptr]
        for i in range(1, nvrow):
            if min_x > buf[inrow_ptr + i*3]:
                min_x = buf[inrow_ptr + i*3]
            if max_x < buf[inrow_ptr + i*3]:
                max_x = buf[inrow_ptr + i*3]
        x0 = int((min_x - bmin[0])*ics)
        x1 = int((max_x - bmin[0])*ics)
        x0 = clamp(x0, 0, w - 1)
        x1 = clamp(x1, 0, w - 1)

        nv: int = nvrow
        nv2: int = nvrow

        for x in range(x0, x1 + 1):
            # Clip polygon to column. store the remaining polygon as well
            cx: float = bmin[0] + x*cs
            nv, nv2 = divide_poly(buf, inrow_ptr, nv2, p1_ptr, p2_ptr, cx+cs, 0)
            inrow_ptr, p2_ptr = p2_ptr, inrow_ptr
            if nv < 3:
                continue

            # Calculate min and max of the span
            smin: float = buf[p1_ptr + 1]
            smax: float = buf[p1_ptr + 1]
            for i in range(1, nv):
                smin = min(smin, buf[p1_ptr + i*3 + 1])
                smax = max(smax, buf[p1_ptr + i*3 + 1])
            smin -= bmin[1]
            smax -= bmin[1]
            # Skip the span if it is outside the heightfield bbox
            if smax < 0.0:
                continue
            if smin > by:
                continue
            # Clamp the span to the heightfield bbox
            if smin < 0.0:
                smin = 0.0
            if smax > by:
                smax = by

            # Snap the span to the heightfield height grid
            ismin = clamp(math.floor(smin*ich), 0, RC_SPAN_MAX_HEIGHT)
            ismax = clamp(math.ceil(smax*ich), ismin + 1, RC_SPAN_MAX_HEIGHT)

            is_add_span = add_span(hf, x, y, ismin, ismax, area, flag_merge_thr)
            if not is_add_span:
                return False

    return True

# Spans will only be added for triangles that overlap the heightfield grid
def rasterize_triangles(verts,  # List of floats
                        tris,  # List of ints
                        areas: List[int],
                        nt: int,
                        solid: Heightfield,  # this is output
                        flag_merge_thr: int) -> bool:
    ics: float = 1.0 / solid.cs
    ich: float = 1.0 / solid.ch
    for i in range(nt):
        i0: int = tris[3*i]
        i1: int = tris[3*i+1]
        i2: int = tris[3*i+2]
        v0: List[float] = verts[3*i0 : 3*i0+3]
        v1: List[float] = verts[3*i1 : 3*i1+3]
        v2: List[float] = verts[3*i2 : 3*i2+3]
        is_rasterize: bool = rasterize_tri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flag_merge_thr)
        if not is_rasterize:
            print("[Navmesh Baker] rasterize_triangles: Fails to rasterize triangles")
            return False

    return True
