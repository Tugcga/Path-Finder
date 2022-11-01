from typing import List, Tuple, Optional
from functools import cmp_to_key
from pathfinder.navmesh_baker.rc_classes import CompactHeightfield, ContourSet, Contour, CompactCell, CompactSpan, ContourRegion, ContourHole, PotentialDiagonal
from pathfinder.navmesh_baker.rc_constants import RC_CONTOUR_TESS_WALL_EDGES, RC_BORDER_REG, RC_NOT_CONNECTED, RC_BORDER_VERTEX, RC_AREA_BORDER, RC_CONTOUR_REG_MASK, RC_CONTOUR_TESS_AREA_EDGES
from pathfinder.navmesh_baker.rc_calcs import get_con, get_dir_offset_x, get_dir_offset_y, v_equal, next, prev

def get_corner_height(x: int,
                      y: int,
                      i: int,
                      dir: int,
                      chf: CompactHeightfield,
                      is_border_vertex: bool) -> Tuple[int, bool]:  # also return is_border_vertex
    s: Optional[CompactSpan] = chf.spans[i]
    ch: int = 0
    ax: int = 0
    ay: int = 0
    ai: int = 0
    asp: Optional[CompactSpan] = None
    ax2: int = 0
    ay2: int = 0
    ai2: int = 0
    as2: Optional[CompactSpan] = None
    if s is not None:
        ch = s.y
        dirp: int = (dir+1) & 0x3
        
        regs: List[int] = [0, 0, 0, 0]
        
        # Combine region and area codes in order to prevent
        # border vertices which are in between two areas to be removed
        regs[0] = s.reg | (chf.areas[i] << 16)
        
        if get_con(s, dir) != RC_NOT_CONNECTED:
            ax = x + get_dir_offset_x(dir)
            ay = y + get_dir_offset_y(dir)
            c: Optional[CompactCell] = chf.cells[ax+ay*chf.width]
            if c is not None:
                ai = c.index + get_con(s, dir)
                asp = chf.spans[ai]
                if asp is not None:
                    ch = max(ch, asp.y)
                    regs[1] = asp.reg | (chf.areas[ai] << 16)
                    if get_con(asp, dirp) != RC_NOT_CONNECTED:
                        ax2 = ax + get_dir_offset_x(dirp)
                        ay2 = ay + get_dir_offset_y(dirp)
                        c2: Optional[CompactCell] = chf.cells[ax2+ay2*chf.width]
                        if c2 is not None:
                            ai2 = c2.index + get_con(asp, dirp)
                            as2 = chf.spans[ai2]
                            if as2 is not None:
                                ch = max(ch, as2.y)
                                regs[2] = as2.reg | (chf.areas[ai2] << 16)

        if get_con(s, dirp) != RC_NOT_CONNECTED:
            ax = x + get_dir_offset_x(dirp)
            ay = y + get_dir_offset_y(dirp)
            cc: Optional[CompactCell] = chf.cells[ax+ay*chf.width]
            if cc is not None:
                ai = cc.index + get_con(s, dirp)
                asp = chf.spans[ai]
                if asp is not None:
                    ch = max(ch, asp.y)
                    regs[3] = asp.reg | (chf.areas[ai] << 16)
                    if get_con(asp, dir) != RC_NOT_CONNECTED:
                        ax2 = ax + get_dir_offset_x(dir)
                        ay2 = ay + get_dir_offset_y(dir)
                        ccc: Optional[CompactCell] = chf.cells[ax2+ay2*chf.width]
                        if ccc is not None:
                            ai2 = ccc.index + get_con(asp, dir)
                            as2 = chf.spans[ai2]
                            if as2 is not None:
                                ch = max(ch, as2.y)
                                regs[2] = as2.reg | (chf.areas[ai2] << 16)

        # Check if the vertex is special edge vertex, these vertices will be removed later
        for j in range(4):
            av: int = j
            bv: int = (j+1) & 0x3
            cv: int = (j+2) & 0x3
            dv: int = (j+3) & 0x3
            
            # The vertex is a border vertex there are two same exterior cells in a row,
            # followed by two interior cells and none of the regions are out of bounds
            two_same_exts: bool = (regs[av] & regs[bv] & RC_BORDER_REG) != 0 and regs[av] == regs[bv]
            two_ints: bool = ((regs[cv] | regs[dv]) & RC_BORDER_REG) == 0
            ints_same_area: bool = (regs[cv]>>16) == (regs[dv]>>16)
            no_zeros: bool = regs[av] != 0 and regs[bv] != 0 and regs[cv] != 0 and regs[dv] != 0
            if two_same_exts and two_ints and ints_same_area and no_zeros:
                is_border_vertex = True
                break
    
    return (ch, is_border_vertex)

def walk_contour(x: int,
                 y: int,
                 i: int,
                 chf: CompactHeightfield,
                 flags: List[int],
                 points: List[int]):
    # Choose the first non-connected edge
    dir: int = 0  # 1 byte
    while (flags[i] & (1 << dir)) == 0:
        dir += 1

    start_dir: int = dir  # 1 byte
    starti: int = i

    area: int = chf.areas[i]  # 1 byte

    iter: int = 0
    s: Optional[CompactSpan] = None
    while iter < 40000:
        iter += 1
        if flags[i] & (1 << dir):
            # Choose the edge corner
            is_border_vertex: bool = False
            is_area_border: bool = False
            px: int = x
            py, is_border_vertex = get_corner_height(x, y, i, dir, chf, is_border_vertex)
            pz: int = y
            if dir == 0:
                pz += 1
            elif dir == 1:
                px += 1
                pz += 1
            elif dir == 2:
                px += 1
            r: int = 0
            s = chf.spans[i]
            if s is not None:
                if get_con(s, dir) != RC_NOT_CONNECTED:
                    ax: int = x + get_dir_offset_x(dir)
                    ay: int = y + get_dir_offset_y(dir)
                    c: Optional[CompactCell] = chf.cells[ax + ay*chf.width]
                    if c is not None:
                        ai: int = c.index + get_con(s, dir)
                        ss: Optional[CompactSpan] = chf.spans[ai]
                        if ss is not None:
                            r = ss.reg
                        if area != chf.areas[ai]:
                            is_area_border = True
                    if is_border_vertex:
                        r |= RC_BORDER_VERTEX
                    if is_area_border:
                        r |= RC_AREA_BORDER
                    points.append(px)
                    points.append(py)
                    points.append(pz)
                    points.append(r)

                    flags[i] &= ~(1 << dir)
                    dir = (dir + 1) & 0x3
        else:
            ni: int = -1
            nx: int = x + get_dir_offset_x(dir)
            ny: int = y + get_dir_offset_y(dir)
            s = chf.spans[i]
            if s is not None:
                if get_con(s, dir) != RC_NOT_CONNECTED:
                    nc: Optional[CompactCell] = chf.cells[nx + ny*chf.width]
                    if nc is not None:
                        ni = nc.index + get_con(s, dir)
            if ni == -1:
                # Should not happen
                return None
            x = nx
            y = ny
            i = ni
            dir = (dir + 3) & 0x3

        if starti == i and start_dir == dir:
            break

def distance_pt_seg(x: int,
                    z: int,
                    px: int,
                    pz: int,
                    qx: int,
                    qz: int) -> float:
    pqx: float = float(qx - px)
    pqz: float = float(qz - pz)
    dx: float = float(x - px)
    dz: float = float(z - pz)
    d: float = pqx*pqx + pqz*pqz
    t: float = pqx*dx + pqz*dz
    if d > 0.0:
        t /= d
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    
    dx = float(px) + t*pqx - float(x)
    dz = float(pz) + t*pqz - float(z)
    
    return dx*dx + dz*dz

def simplify_contour(points: List[int],
                     simplified: List[int],
                     max_error: float,
                     max_edge_len: int,
                     build_flags: int):
    # Add initial points
    has_connections: bool = False
    i: int = 0
    while i < len(points):
        if points[i + 3] & RC_CONTOUR_REG_MASK != 0:
            has_connections = True
            break
        i += 4

    ii: int = 0
    ax: int = 0
    az: int = 0
    ai: int = 0
    bx: int = 0
    bz: int = 0
    bi: int = 0
    
    # Find maximum deviation from the segment
    maxi: int = -1
    pn: int = len(points) // 4
    ci: int = (ai+1) % pn

    if has_connections:
        # The contour has some portals to other regions
        # Add a new point to every location where the region changes
        ni: int = len(points) // 4
        i = 0
        while i < ni:
            ii = (i + 1) % ni
            different_regs: bool = (points[i*4+3] & RC_CONTOUR_REG_MASK) != (points[ii*4+3] & RC_CONTOUR_REG_MASK)
            area_borders: bool = (points[i*4+3] & RC_AREA_BORDER) != (points[ii*4+3] & RC_AREA_BORDER)
            if different_regs or area_borders:
                simplified.append(points[i*4])
                simplified.append(points[i*4 + 1])
                simplified.append(points[i*4 + 2])
                simplified.append(i)
            i += 1

    if len(simplified) == 0:
        # If there is no connections at all,
        # create some initial points for the simplification process
        # Find lower-left and upper-right vertices of the contour
        llx: int = points[0]
        lly: int = points[1]
        llz: int = points[2]
        lli: int = 0
        urx: int = points[0]
        ury: int = points[1]
        urz: int = points[2]
        uri: int = 0
        i = 0
        while i < len(points):
            x: int = points[i+0]
            y: int = points[i+1]
            z: int = points[i+2]
            if (x < llx or (x == llx and z < llz)):
                llx = x
                lly = y
                llz = z
                lli = i // 4
            if (x > urx or (x == urx and z > urz)):
                urx = x
                ury = y
                urz = z
                uri = i // 4
            i += 4
        simplified.append(llx)
        simplified.append(lly)
        simplified.append(llz)
        simplified.append(lli)
        
        simplified.append(urx)
        simplified.append(ury)
        simplified.append(urz)
        simplified.append(uri)

    # Add points until all raw points are within
    # error tolerance to the simplified shape
    i = 0
    n: int = 0
    while i < len(simplified) // 4:
        ii = (i+1) % (len(simplified) // 4)
        
        ax = simplified[i*4+0]
        az = simplified[i*4+2]
        ai = simplified[i*4+3]

        bx = simplified[ii*4+0]
        bz = simplified[ii*4+2]
        bi = simplified[ii*4+3]

        # Find maximum deviation from the segment
        maxd: float = 0.0
        maxi = -1
        ci = 0
        cinc: int = 0
        endi: int = 0

        # Traverse the segment in lexilogical order so that the
        # max deviation is calculated similarly when traversing
        # opposite segments
        if (bx > ax or (bx == ax and bz > az)):
            cinc = 1
            ci = (ai+cinc) % pn
            endi = bi
        else:
            cinc = pn-1
            ci = (bi+cinc) % pn
            endi = ai
            ax, bx = bx, ax
            az, bz = bz, az

        # Tessellate only outer edges or edges between areas
        if ((points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0 or (points[ci*4+3] & RC_AREA_BORDER)):
            while (ci != endi):
                d: float = distance_pt_seg(points[ci*4+0], points[ci*4+2], ax, az, bx, bz)
                if d > maxd:
                    maxd = d
                    maxi = ci
                ci = (ci+cinc) % pn

        # If the max deviation is larger than accepted error,
        # add new point, else continue to next segment
        if maxi != -1 and maxd > (max_error*max_error):
            # Add space for the new point
            simplified.extend([0]*4)
            n = len(simplified) // 4
            for j in range(n - 1, i, -1):
                simplified[j*4+0] = simplified[(j-1)*4+0]
                simplified[j*4+1] = simplified[(j-1)*4+1]
                simplified[j*4+2] = simplified[(j-1)*4+2]
                simplified[j*4+3] = simplified[(j-1)*4+3]
            # Add the point
            simplified[(i+1)*4+0] = points[maxi*4+0]
            simplified[(i+1)*4+1] = points[maxi*4+1]
            simplified[(i+1)*4+2] = points[maxi*4+2]
            simplified[(i+1)*4+3] = maxi
        else:
            i += 1

    # Split too long edges
    if max_edge_len > 0 and (build_flags & (RC_CONTOUR_TESS_WALL_EDGES | RC_CONTOUR_TESS_AREA_EDGES)) != 0:
        i = 0
        while i < len(simplified) // 4:
            ii = (i+1) % (len(simplified) // 4)
            
            ax = simplified[i*4+0]
            az = simplified[i*4+2]
            ai = simplified[i*4+3]
            
            bx = simplified[ii*4+0]
            bz = simplified[ii*4+2]
            bi = simplified[ii*4+3]
            
            # Find maximum deviation from the segment
            maxi = -1
            ci = (ai+1) % pn
            
            # Tessellate only outer edges or edges between areas
            tess: bool = False
            # Wall edges
            if (build_flags & RC_CONTOUR_TESS_WALL_EDGES) and (points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0:
                tess = True
            # Edges between areas
            if (build_flags & RC_CONTOUR_TESS_AREA_EDGES) and (points[ci*4+3] & RC_AREA_BORDER):
                tess = True
            
            if tess:
                dx: int = bx - ax
                dz: int = bz - az
                if dx*dx + dz*dz > max_edge_len*max_edge_len:
                    # Round based on the segments in lexilogical order so that the
                    # max tesselation is consistent regardles in which direction
                    # segments are traversed
                    n = (bi+pn - ai) if bi < ai else (bi - ai)
                    if n > 1:
                        if bx > ax or (bx == ax and bz > az):
                            maxi = (ai + n // 2) % pn
                        else:
                            maxi = (ai + (n+1) // 2) % pn
            
            # If the max deviation is larger than accepted error,
            # add new point, else continue to next segment
            if maxi != -1:
                # Add space for the new point
                simplified.extend([0]*4)
                n = len(simplified) // 4
                for j in range(n - 1, i, -1):
                    simplified[j*4+0] = simplified[(j-1)*4+0]
                    simplified[j*4+1] = simplified[(j-1)*4+1]
                    simplified[j*4+2] = simplified[(j-1)*4+2]
                    simplified[j*4+3] = simplified[(j-1)*4+3]
                # Add the point
                simplified[(i+1)*4+0] = points[maxi*4+0]
                simplified[(i+1)*4+1] = points[maxi*4+1]
                simplified[(i+1)*4+2] = points[maxi*4+2]
                simplified[(i+1)*4+3] = maxi
            else:
                i += 1
    
    i = 0
    while i < len(simplified) // 4:
        # The edge vertex flag is take from the current raw point,
        # and the neighbour region is take from the next raw point
        ai = (simplified[i*4+3]+1) % pn
        bi = simplified[i*4+3]
        simplified[i*4+3] = (points[ai*4+3] & (RC_CONTOUR_REG_MASK|RC_AREA_BORDER)) | (points[bi*4+3] & RC_BORDER_VERTEX)
        i += 1

def remove_degenerate_segments(simplified: List[int]):
    # Remove adjacent vertices which are equal on xz-plane,
    # or else the triangulator will get confused
    npts: int = len(simplified) // 4
    i: int = 0
    while i < npts:
        ni: int = next(i, npts)
        if v_equal(4*i, 4*ni, simplified):
            # Degenerate segment, remove
            for j in range(i, len(simplified) // 4 - 1):
                simplified[j*4+0] = simplified[(j+1)*4+0]
                simplified[j*4+1] = simplified[(j+1)*4+1]
                simplified[j*4+2] = simplified[(j+1)*4+2]
                simplified[j*4+3] = simplified[(j+1)*4+3]
            simplified.pop(-1)
            simplified.pop(-1)
            simplified.pop(-1)
            simplified.pop(-1)
            npts -= 1
        i += 1

def calc_area_of_polygon_2d(verts: List[int],
                            nverts: int) -> int:
    area: int = 0
    j: int = nverts - 1
    for i in range(0, nverts):
        area += verts[4*i] * verts[4*j + 2] - verts[4*j] * verts[4*i + 2]
        j = i
    return (area + 1) // 2

def find_left_most_vertex(contour: Contour) -> Tuple[int, int, int]:
    # return minx, minz, leftmost
    minx: int = contour.verts[0]
    minz: int = contour.verts[2]
    leftmost: int = 0
    for i in range(1, contour.nverts):
        x: int = contour.verts[i*4]
        z: int = contour.verts[i*4 + 2]
        if x < minx or (x == minx and z < minz):
            minx = x
            minz = z
            leftmost = i
    return (minx, minz, leftmost)

def compare_holes(a: ContourHole,
                  b: ContourHole) -> int:
    if a.minx == b.minx:
        if a.minz < b.minz:
            return -1
        if a.minz > b.minz:
            return 1
    else:
        if a.minx < b.minx:
            return -1
        if a.minx > b.minx:
            return 1
    return 0

def area2(array: List[int],
          a: int,
          b: int,
          c: int) -> int:
    return (array[b] - array[a]) * (array[c + 2] - array[a + 2]) - (array[c] - array[a]) * (array[b + 2] - array[a + 2])

def left_on(array: List[int],
            a: int,
            b: int,
            c: int) -> bool:
    return area2(array, a, b, c) <= 0

def left(a: int,
         a_array: List[int],
         b: int,
         b_array: List[int],
         c: int,
         c_array: List[int]) -> bool:
    v: int = (b_array[b]-a_array[a])*(c_array[c+2]-a_array[a+2]) - (c_array[c]-a_array[a])*(b_array[b+2]-a_array[a+2])
    return v < 0

def xorb(x: bool,
         y: bool) -> bool:
    return (not x) ^ (not y)

def in_cone(i: int,
            n: int,
            verts: List[int],
            pj: int,  # pointer to the next array
            p_array: List[int]) -> bool:
    pi: int = i * 4  # pointer to verts array
    pi1: int = next(i, n) * 4
    pin1: int = prev(i, n) * 4

    v1: int = (p_array[pj]-verts[pi])*(verts[pin1+2]-verts[pi+2]) - (verts[pin1]-verts[pi])*(p_array[pj+2]-verts[pi+2])
    v2: int = (verts[pi]-p_array[pj])*(verts[pi1+2]-p_array[pj+2]) - (verts[pi1]-p_array[pj])*(verts[pi+2]-p_array[pj+2])
    if left_on(verts, pin1, pi, pi1):
        return v1 < 0 and v2 < 0
    return not (v1 <= 0 and v2 <= 0)

def compare_diag_dist(a: PotentialDiagonal,
                      b: PotentialDiagonal) -> int:
    if a.dist < b.dist:
        return -1
    if a.dist > b.dist:
        return 1
    return 0

def collinear(a: int,
              a_array: List[int],
              b: int,
              b_array: List[int],
              c: int,
              c_array: List[int]) -> bool:
    v: int = (b_array[b]-a_array[a])*(c_array[c+2]-a_array[a+2]) - (c_array[c]-a_array[a])*(b_array[b+2]-a_array[a+2])
    return v == 0

def intersect_prop(a: int,
                   a_array: List[int],
                   b: int,
                   b_array: List[int],
                   c: int,
                   c_array: List[int],
                   d: int,
                   d_array: List[int]) -> bool:
    if collinear(a, a_array, b, b_array, c, c_array) or collinear(a, a_array, b, b_array, d, d_array) or collinear(c, c_array, d, d_array, a, a_array) or collinear(c, c_array, d, d_array, b, b_array):
        return False
    return xorb(left(a, a_array, b, b_array, c, c_array), left(a, a_array, b, b_array, d, d_array)) and xorb(left(c, c_array, d, d_array, a, a_array), left(c, c_array, d, d_array, b, b_array))

def between(a: int,
            a_array: List[int],
            b: int,
            b_array: List[int],
            c: int,
            c_array: List[int]) -> bool:
    if not collinear(a, a_array, b, b_array, c, c_array):
        return False
    if a_array[a] != b_array[b]:
        return ((a_array[a] <= c_array[c]) and (c_array[c] <= b_array[b])) or ((a_array[a] >= c_array[c]) and (c_array[c] >= b_array[b]))
    else:
        return ((a_array[a+2] <= c_array[c+2]) and (c_array[c+2] <= b_array[b+2])) or ((a_array[a+2] >= c_array[c+2]) and (c_array[c+2] >= b_array[b+2]))

def intersect(a: int,
              a_array: List[int],
              b: int,
              b_array: List[int],
              c: int,
              c_array: List[int],
              d: int,
              d_array: List[int]) -> bool:
    if intersect_prop(a, a_array, b, b_array, c, c_array, d, d_array):
        return True
    elif between(a, a_array, b, b_array, c, c_array) or between(a, a_array, b, b_array, d, d_array) or between(c, c_array, d, d_array, a, a_array) or between(c, c_array, d, d_array, b, b_array):
        return True
    else:
        return False

def intersect_seg_countour(d0: int,
                           d0_array: List[int],
                           d1: int,
                           d1_array: List[int],
                           i: int,
                           n: int,
                           verts: List[int]) -> bool:
    for k in range(n):
        k1: int = next(k, n)
        if i == k or i == k1:
            continue
        p0: int = k*4  # poitners to verts
        p1: int = k1*4
        if (d0_array[d0] == verts[p0] and d0_array[d0+2] == verts[p0+2]) or (d1_array[d1] == verts[p0] and d1_array[d1+2] == verts[p0+2]) or (d0_array[d0] == verts[p1] and d0_array[d0+2] == verts[p1+2]) or (d1_array[d1] == verts[p1] and d1_array[d1+2] == verts[p1+2]):
            continue
        if intersect(d0, d0_array, d1, d1_array, p0, verts, p1, verts):
            return True
    return False

def merge_contours(ca: Contour,
                   cb: Contour,
                   ia: int,
                   ib: int) -> bool:
    maxVerts: int = ca.nverts + cb.nverts + 2
    verts: List[int] = [0] * (maxVerts*4)
    
    nv: int = 0
    dst: int = 0
    src: int = 0
    
    # Copy contour A
    for i in range(ca.nverts + 1):
        dst = nv*4  # pointer to verts
        src = ((ia+i)%ca.nverts)*4  # pointer to ca.verts
        verts[dst] = ca.verts[src]
        verts[dst + 1] = ca.verts[src + 1]
        verts[dst + 2] = ca.verts[src + 2]
        verts[dst + 3] = ca.verts[src + 3]
        nv += 1

    # Copy contour B
    for i in range(cb.nverts + 1):
        dst = nv*4
        src = ((ib+i)%cb.nverts)*4  # pointer to cb.verts
        verts[dst] = cb.verts[src]
        verts[dst + 1] = cb.verts[src + 1]
        verts[dst + 2] = cb.verts[src + 2]
        verts[dst + 3] = cb.verts[src + 3]
        nv += 1

    ca.verts = verts
    ca.nverts = nv

    cb.verts = []
    cb.nverts = 0
    
    return True

def merge_region_holes(region: ContourRegion, holes: List[ContourHole]):
    # Sort holes from left to right
    contour: Optional[Contour] = None
    for i in range(region.nholes):
        contour = holes[region.holes_index + i].contour
        if contour is not None:
            (minx, minz, leftmost) = find_left_most_vertex(contour)
            holes[region.holes_index + i].minx = minx
            holes[region.holes_index + i].minz = minz
            holes[region.holes_index + i].leftmost = leftmost
    sorted_holes: List[ContourHole] = sorted(holes[region.holes_index:region.holes_index+region.nholes], key=cmp_to_key(compare_holes))
    for i in range(region.nholes):
        holes[region.holes_index + i] = sorted_holes[i]

    reg_outline: Optional[Contour] = region.outline
    max_verts: int = 0
    if reg_outline is not None:
        max_verts = reg_outline.nverts
        for i in range(region.nholes):
            contour = holes[region.holes_index + i].contour
            if contour is not None:
                max_verts += contour.nverts

    diags: List[PotentialDiagonal] = [PotentialDiagonal() for _ in range(max_verts)]

    outline: Optional[Contour] = region.outline
    if outline is not None:
        # Merge holes into the outline one by one.
        for i in range(region.nholes):
            hole: Optional[Contour] = holes[region.holes_index].contour
            
            index: int = -1
            bestVertex: int = holes[region.holes_index + i].leftmost
            if hole is not None:
                for iter in range(hole.nverts):
                    # Find potential diagonals
                    # The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline
                    # ..o j-1
                    #   |
                    #   |   * best
                    #   |
                    # j o-----o j+1
                    #         :
                    ndiags: int = 0
                    corner: int = bestVertex*4  # pointer to hole.verts
                    for j in range(outline.nverts):
                        if in_cone(j, outline.nverts, outline.verts, corner, hole.verts):
                            dx: int = outline.verts[j*4+0] - hole.verts[corner]
                            dz: int = outline.verts[j*4+2] - hole.verts[corner + 2]
                            diags[ndiags].vert = j
                            diags[ndiags].dist = dx*dx + dz*dz
                            ndiags += 1

                    # Sort potential diagonals by distance, we want to make the connection as short as possible
                    sorted_diags: List[PotentialDiagonal] = sorted(diags[0:ndiags], key=cmp_to_key(compare_diag_dist))
                    for m in range(ndiags):
                        diags[m] = sorted_diags[m]
                    
                    # Find a diagonal that is not intersecting the outline not the remaining holes
                    index = -1
                    for j in range(ndiags):
                        pt: int = diags[j].vert*4  # pointer to outline.verts
                        intersect: bool = intersect_seg_countour(pt, outline.verts, corner, hole.verts, diags[i].vert, outline.nverts, outline.verts)
                        
                        k: int = i
                        while k < region.nholes and not intersect:
                            contour = holes[region.holes_index + k].contour
                            if contour is not None:
                                intersect |= intersect_seg_countour(pt, outline.verts, corner, hole.verts, -1, contour.nverts, contour.verts)
                            k += 1
                        if not intersect:
                            index = diags[j].vert
                            break
                    # If found non-intersecting diagonal, stop looking
                    if index != -1:
                        break
                    # All the potential diagonals for the current vertex were intersecting, try next vertex
                    bestVertex = (bestVertex + 1) % hole.nverts

                if index == -1:
                    print("[Navmesh Baker] merge_holes: Failed to find merge points for " + str(region.outline) + " and " + str(hole) +".")
                    continue
                if not merge_contours(outline, hole, index, bestVertex):
                    print("[Navmesh Baker] merge_holes: Failed to merge contours " + str(region.outline) + " and " + str(hole) + ".")
                    continue

def build_contours(chf: CompactHeightfield,
                   max_error: float,
                   max_edge_len: int,
                   cset: ContourSet,
                   build_flags: int = RC_CONTOUR_TESS_WALL_EDGES) -> bool:
    w: int = chf.width
    h: int = chf.height
    border_size: int = chf.border_size

    bmin = [chf.bmin[0], chf.bmin[1], chf.bmin[2]]
    bmax = [chf.bmax[0], chf.bmax[1], chf.bmax[2]]
    if border_size > 0:
        # If the heightfield was build with bordersize, remove the offset
        pad: float = border_size * chf.cs
        bmin[0] += pad
        bmin[2] += pad
        bmax[0] -= pad
        bmax[2] -= pad
    cset.bmin = (bmin[0], bmin[1], bmin[2])
    cset.bmax = (bmax[0], bmax[1], bmax[2])
    cset.cs = chf.cs
    cset.ch = chf.ch
    cset.width = chf.width - chf.border_size * 2
    cset.height = chf.height - chf.border_size * 2
    cset.border_size = chf.border_size
    cset.max_error = max_error

    max_contours = max(chf.max_regions, 8)
    cset.conts = [Contour() for _ in range(max_contours)]
    cset.nconts = 0

    flags: List[int] = [0] * chf.span_count  # 1 byte per element

    c: Optional[CompactCell] = None
    s: Optional[CompactSpan] = None
    # Mark boundaries
    for y in range(h):
        for x in range(w):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    res: int = 0  # 1 byte
                    s = chf.spans[i]
                    if s is not None:
                        if (s.reg == 0) or (s.reg & RC_BORDER_REG):
                            flags[i] = 0
                            continue
                        for dir in range(4):
                            r: int = 0  # 2 bytes
                            if get_con(s, dir) != RC_NOT_CONNECTED:
                                ax: int = x + get_dir_offset_x(dir)
                                ay: int = y + get_dir_offset_y(dir)
                                cc: Optional[CompactCell] = chf.cells[ax + ay*w]
                                if cc is not None:
                                    ai: int = cc.index + get_con(s, dir)
                                    ss: Optional[CompactSpan] = chf.spans[ai]
                                    if ss is not None:
                                        r = ss.reg
                            if r == s.reg:
                                res |= (1 << dir)
                    flags[i] = res ^ 0xf  # Inverse, mark non connected edges

    verts: List[int] = [0] * 256  # <--- does not need this
    simplified: List[int] = [0] * 64

    for y in range(h):
        for x in range(w):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    if flags[i] == 0 or flags[i] == 0xf:
                        flags[i] = 0
                        continue
                    s = chf.spans[i]
                    if s is not None:
                        reg: int = s.reg  # 2 bytes
                        if (reg == 0) or (reg & RC_BORDER_REG):
                            continue
                        area: int = chf.areas[i]  # 1 byte

                        verts.clear()
                        simplified.clear()

                        walk_contour(x, y, i, chf, flags, verts)
                        simplify_contour(verts, simplified, max_error, max_edge_len, build_flags)
                        remove_degenerate_segments(simplified)

                        # Store region->contour remap info
                        # Create contour
                        if len(simplified) // 4 >= 3:
                            if cset.nconts > max_contours:
                                # Allocate more contours
                                # This happens when a region has holes
                                old_max: int = max_contours
                                max_contours *= 2
                                new_conts: List[Contour] = [Contour() for _ in range(max_contours)]
                                for j in range(cset.nconts):
                                    new_conts[j] = cset.conts[j]
                                    # Reset source pointers to prevent data deletion
                                    cset.conts[j].verts = []
                                    cset.conts[j].rverts = []
                                cset.conts = new_conts

                            cont: Contour = cset.conts[cset.nconts]
                            cset.nconts += 1

                            cont.nverts = len(simplified) // 4
                            cont.verts = [0] * (cont.nverts * 4)
                            for j in range(len(cont.verts)):
                                cont.verts[j] = simplified[j]
                            if border_size > 0:
                                # If the heightfield was build with bordersize, remove the offset
                                for j in range(cont.nverts):
                                    cont.verts[4*j] -= border_size
                                    cont.verts[4*j + 2] -= border_size
                            cont.nrverts = len(verts) // 4
                            cont.rverts = [0] * (cont.nrverts * 4)
                            for j in range(len(cont.rverts)):
                                cont.rverts[j] = verts[j]
                            if border_size > 0:
                                # If the heightfield was build with bordersize, remove the offset
                                for j in range(cont.nrverts):
                                    cont.rverts[4*j] -= border_size
                                    cont.rverts[4*j + 2] -= border_size
                            cont.reg = reg
                            cont.area = area

    # Merge holes if needed
    if cset.nconts > 0:
        # Calculate winding of all polygons
        winding: List[int] = [0] * cset.nconts  # 2 bytes (signed char)
        nholes: int = 0
        for i in range(cset.nconts):
            cont1: Contour = cset.conts[i]
            # If the contour is wound backwards, it is a hole
            winding[i] = -1 if calc_area_of_polygon_2d(cont1.verts, cont1.nverts) < 0 else 1
            if winding[i] < 0:
                nholes += 1

        if nholes > 0:
            # Collect outline contour and holes contours per region
            # We assume that there is one outline and multiple holes
            nregions: int = chf.max_regions + 1
            regions: List[ContourRegion] = [ContourRegion() for _ in range(nregions)]
            holes: List[ContourHole] = [ContourHole() for _ in range(cset.nconts)]

            for i in range(cset.nconts):
                cont2: Contour = cset.conts[i]
                # Positively would contours are outlines, negative holes
                if winding[i] > 0:
                    if regions[cont2.reg].outline:
                        print("[Navmesh Baker] build_contours: Multiple outlines for region " + str(cont2.reg))
                    regions[cont2.reg].outline = cont2
                else:
                    regions[cont2.reg].nholes += 1
            index: int = 0
            for i in range(nregions):
                if regions[i].nholes > 0:
                    regions[i].holes_index = index
                    index += regions[i].nholes
                    regions[i].nholes = 0
            for i in range(cset.nconts):
                cont3: Contour = cset.conts[i]
                co_reg: ContourRegion = regions[cont3.reg]
                if winding[i] < 0:
                    holes[co_reg.holes_index + co_reg.nholes].contour = cont3
                    co_reg.nholes += 1

            # Finally merge each regions holes into the outline
            for i in range(nregions):
                c_reg: ContourRegion = regions[i]
                if c_reg.nholes == 0:
                    continue
                if c_reg.outline is not None:
                    merge_region_holes(c_reg, holes)
                else:
                    # The region does not have an outline
                    # This can happen if the contour becomes selfoverlapping because of
                    # too aggressive simplification settings
                    print("[Navmesh Baker] build_contours: Bad outline for region " + str(i) + ", contour simplification is likely too aggressive")
    return True
