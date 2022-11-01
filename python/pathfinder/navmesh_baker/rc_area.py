from typing import List, Tuple, Optional
from pathfinder.navmesh_baker.rc_classes import CompactHeightfield, CompactCell, CompactSpan, LevelStackEntry, DirtyEntry, Region
from pathfinder.navmesh_baker.rc_constants import RC_NULL_AREA, RC_NOT_CONNECTED, RC_BORDER_REG
from pathfinder.navmesh_baker.rc_calcs import get_con, get_dir_offset_x, get_dir_offset_y

def erode_walkable_area(radius: int,
                        chf: CompactHeightfield) -> bool:
    w: int = chf.width
    h: int = chf.height

    # Init distance
    dist: List[int] = [255] * chf.span_count

    # Mark boundary cells
    s: Optional[CompactSpan] = None
    c: Optional[CompactCell] = None
    c2: Optional[CompactCell] = None
    c3: Optional[CompactCell] = None
    asp: Optional[CompactSpan] = None
    ax: int = 0
    ay: int = 0
    ai: int = 0
    aax: int = 0
    aay: int = 0
    aai: int = 0
    for y in range(h):
        for x in range(w):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    if chf.areas[i] == RC_NULL_AREA:
                        dist[i] = 0
                    else:
                        s = chf.spans[i]
                        nc: int = 0
                        if s is not None:
                            for dir in range(4):
                                if get_con(s, dir) != RC_NOT_CONNECTED:
                                    nx: int = x + get_dir_offset_x(dir)
                                    ny: int = y + get_dir_offset_y(dir)
                                    c = chf.cells[nx + ny*w]
                                    if c is not None:
                                        nidx: int = c.index + get_con(s, dir)
                                        if chf.areas[nidx] != RC_NULL_AREA:
                                            nc += 1
                        # At least one missing neighbour
                        if nc != 4:
                            dist[i] = 0

    nd: int = 0  # 1 byte

    # Pass 1
    for y in range(h):
        for x in range(w):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    s= chf.spans[i]
                    if s is not None:
                        if get_con(s, 0) != RC_NOT_CONNECTED:
                            # (-1, 0)
                            ax = x + get_dir_offset_x(0)
                            ay = y + get_dir_offset_y(0)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 0)
                                asp = chf.spans[ai]
                                nd = min(dist[ai] + 2, 255)
                                if nd < dist[i]:
                                    dist[i] = nd

                                # (-1, -1)
                                if asp is not None:
                                    if get_con(asp, 3) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(3)
                                        aay = ay + get_dir_offset_y(3)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 3)
                                            nd = min(dist[aai] + 3, 255)
                                            if nd < dist[i]:
                                                dist[i] = nd
                        if get_con(s, 3) != RC_NOT_CONNECTED:
                            # (0, -1)
                            ax = x + get_dir_offset_x(3)
                            ay = y + get_dir_offset_y(3)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 3)
                                asp = chf.spans[ai]
                                nd = min(dist[ai] + 2, 255)
                                if nd < dist[i]:
                                    dist[i] = nd

                                # (1, -1)
                                if asp is not None:
                                    if get_con(asp, 2) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(2)
                                        aay = ay + get_dir_offset_y(2)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 2)
                                            nd = min(dist[aai] + 3, 255)
                                            if nd < dist[i]:
                                                dist[i] = nd

    # Phase 2
    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    s = chf.spans[i]

                    if s is not None:
                        if get_con(s, 2) != RC_NOT_CONNECTED:
                            # (1, 0)
                            ax = x + get_dir_offset_x(2)
                            ay = y + get_dir_offset_y(2)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 2)
                                asp = chf.spans[ai]
                                nd = min(dist[ai] + 2, 255)
                                if nd < dist[i]:
                                    dist[i] = nd

                                # (1, 1)
                                if asp is not None:
                                    if get_con(asp, 1) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(1)
                                        aay = ay + get_dir_offset_y(1)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 1)
                                            nd = min(dist[aai] + 3, 255)
                                            if nd < dist[i]:
                                                dist[i] = nd
                        if get_con(s, 1) != RC_NOT_CONNECTED:
                            # (0, 1)
                            ax = x + get_dir_offset_x(1)
                            ay = y + get_dir_offset_y(1)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 1)
                                asp = chf.spans[ai]
                                nd = min(dist[ai] + 2, 255)
                                if nd < dist[i]:
                                    dist[i] = nd

                                if asp is not None:
                                    if get_con(asp, 0) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(0)
                                        aay = ay + get_dir_offset_y(0)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 0)
                                            nd = min(dist[aai] + 3, 255)
                                            if nd < dist[i]:
                                                dist[i] = nd

    thr: int = radius * 2  # 1 byte
    for i in range(chf.span_count):
        if dist[i] < thr:
            chf.areas[i] = RC_NULL_AREA

    return True

def calculate_distance_field(chf: CompactHeightfield,
                             src: List[int],
                             max_dist: int) -> int:  # return new value of the max_dist
    w: int = chf.width
    h: int = chf.height

    # Init distance and points
    for i in range(chf.span_count):
        src[i] = 65535

    # Mark boundary cells
    c: Optional[CompactCell] = None
    c2: Optional[CompactCell] = None
    c3: Optional[CompactCell] = None
    s: Optional[CompactSpan] = None
    asp: Optional[CompactSpan] = None
    ax: int = 0
    ay: int = 0
    ai: int = 0
    aax: int = 0
    aay: int = 0
    aai: int = 0
    for y in range(h):
        for x in range(w):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    s = chf.spans[i]
                    area: int = chf.areas[i]  # 1 byte

                    nc: int = 0
                    if s is not None:
                        for dir in range(4):
                            if get_con(s, dir) != RC_NOT_CONNECTED:
                                ax = x + get_dir_offset_x(dir)
                                ay = y + get_dir_offset_y(dir)
                                c2 = chf.cells[ax + ay*w]
                                if c2 is not None:
                                    ai = c2.index + get_con(s, dir)
                                    if area == chf.areas[ai]:
                                        nc += 1
                    if nc != 4:
                        src[i] = 0

    # Phase 1
    for y in range(h):
        for x in range(w):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    s = chf.spans[i]

                    if s is not None:
                        if get_con(s, 0) != RC_NOT_CONNECTED:
                            # (-1, 0)
                            ax = x + get_dir_offset_x(0)
                            ay = y + get_dir_offset_y(0)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 0)
                                asp = chf.spans[ai]
                                if src[ai] + 2 < src[i]:
                                    src[i] = src[ai] + 2

                                # (-1, -1)
                                if asp is not None:
                                    if get_con(asp, 3) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(3)
                                        aay = ay + get_dir_offset_y(3)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 3)
                                            if src[aai] + 3 < src[i]:
                                                src[i] = src[aai] + 3
                        if get_con(s, 3) != RC_NOT_CONNECTED:
                            # (0, -1)
                            ax = x + get_dir_offset_x(3)
                            ay = y + get_dir_offset_y(3)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 3)
                                asp = chf.spans[ai]
                                if src[ai] + 2 < src[i]:
                                    src[i] = src[ai] + 2

                                # (1, -1)
                                if asp is not None:
                                    if get_con(asp, 2) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(2)
                                        aay = ay + get_dir_offset_y(2)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 2)
                                            if src[aai] + 3 < src[i]:
                                                src[i] = src[aai] + 3

    # Pahse 2
    for y in range(h - 1, -1, -1):
        for x in range(w - 1, -1, -1):
            c = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    s = chf.spans[i]

                    if s is not None:
                        if get_con(s, 2) != RC_NOT_CONNECTED:
                            # (1, 0)
                            ax = x + get_dir_offset_x(2)
                            ay = y + get_dir_offset_y(2)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 2)
                                asp = chf.spans[ai]
                                if src[ai] + 2 < src[i]:
                                    src[i] = src[ai] + 2

                                # (1, 1)
                                if asp is not None:
                                    if get_con(asp, 1) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(1)
                                        aay = ay + get_dir_offset_y(1)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 1)
                                            if src[aai] + 3 < src[i]:
                                                src[i] = src[aai] + 3
                        if get_con(s, 1) != RC_NOT_CONNECTED:
                            # (0, 1)
                            ax = x + get_dir_offset_x(1)
                            ay = y + get_dir_offset_y(1)
                            c2 = chf.cells[ax + ay*w]
                            if c2 is not None:
                                ai = c2.index + get_con(s, 1)
                                asp = chf.spans[ai]
                                if src[ai] + 2 < src[i]:
                                    src[i] = src[ai] + 2

                                # (-1, 1)
                                if asp is not None:
                                    if get_con(asp, 0) != RC_NOT_CONNECTED:
                                        aax = ax + get_dir_offset_x(0)
                                        aay = ay + get_dir_offset_y(0)
                                        c3 = chf.cells[aax + aay*w]
                                        if c3 is not None:
                                            aai = c3.index + get_con(asp, 0)
                                            if src[aai] + 3 < src[i]:
                                                src[i] = src[aai] + 3
    max_dist = 0
    for i in range(chf.span_count):
        max_dist = max(src[i], max_dist)

    return max_dist

def box_blur(chf: CompactHeightfield,
             thr: int,
             src: List[int],
             dst: List[int]):
    w: int = chf.width
    h: int = chf.height

    thr = thr * 2

    for y in range(h):
        for x in range(w):
            c: Optional[CompactCell] = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    s: Optional[CompactSpan] = chf.spans[i]
                    cd: int = src[i]  # 2 bytes
                    if cd <= thr:
                        dst[i] = cd
                        continue

                    d: int = cd
                    for dir in range(4):
                        if s is not None:
                            if get_con(s, dir) != RC_NOT_CONNECTED:
                                ax: int = x + get_dir_offset_x(dir)
                                ay: int = y + get_dir_offset_y(dir)
                                c2: Optional[CompactCell] = chf.cells[ax + ay*w]
                                if c2 is not None:
                                    ai: int = c2.index + get_con(s, dir)
                                    d += src[ai]

                                    asp: Optional[CompactSpan] = chf.spans[ai]
                                    dir2: int = (dir + 1) & 0x3
                                    if asp is not None:
                                        if get_con(asp, dir2) != RC_NOT_CONNECTED:
                                            ax2: int = ax + get_dir_offset_x(dir2)
                                            ay2: int = ay + get_dir_offset_y(dir2)
                                            c3: Optional[CompactCell] = chf.cells[ax2 + ay2*w]
                                            if c3 is not None:
                                                ai2: int = c3.index + get_con(asp, dir2)
                                                d += src[ai2]
                                        else:
                                            d += cd
                            else:
                                d += cd * 2
                    dst[i] = (d + 5) // 9

def build_distance_field(chf: CompactHeightfield) -> bool:
    src: List[int] = [0] * chf.span_count  # 2 bytes per element
    dst: List[int] = [0] * chf.span_count

    max_dist: int = 0  # 2 bytes
    max_dist = calculate_distance_field(chf, src, max_dist)
    chf.max_distance = max_dist

    box_blur(chf, 1, src, dst)
    chf.dist = dst

    return True

def paint_rect_region(minx: int,
                      maxx: int,
                      miny: int,
                      maxy: int,
                      reg_id: int,  # 2 bytes
                      chf: CompactHeightfield,
                      buf: List[int],  # 2 bytes per element
                      src_reg: int):  # src_reg is a pointer to the buffer
    w: int = chf.width
    for y in range(miny, maxy):
        for x in range(minx, maxx):
            c: Optional[CompactCell] = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    if chf.areas[i] != RC_NULL_AREA:
                        buf[src_reg + i] = reg_id

def sort_cells_by_level(start_level: int,
                        chf: CompactHeightfield,
                        buf: List[int],
                        src_reg: int,
                        nb_stacks: int,
                        stacks: List[List[LevelStackEntry]],
                        log_levels_per_stack: int):
    w: int = chf.width
    h: int = chf.height
    start_level = start_level >> log_levels_per_stack

    for j in range(nb_stacks):
        stacks[j].clear()

    # put all cells in the level range into the appropriate stacks
    for y in range(h):
        for x in range(w):
            c: Optional[CompactCell] = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    if chf.areas[i] == RC_NULL_AREA or buf[src_reg + i] != 0:
                        continue

                    level: int = chf.dist[i] >> log_levels_per_stack
                    s_id: int = start_level - level
                    if s_id >= nb_stacks:
                        continue
                    if s_id < 0:
                        s_id = 0

                    stacks[s_id].append(LevelStackEntry(x, y, i))

def append_stacks(src_stack: List[LevelStackEntry],
                  dst_stack: List[LevelStackEntry],
                  buf: List[int],
                  src_reg: int):
    for j in range(len(src_stack)):
        i: int = src_stack[j].index
        if i < 0 or buf[src_reg + i] != 0:
            continue
        dst_stack.append(src_stack[j])

def expand_regions(max_iter: int,
                   level: int,
                   chf: CompactHeightfield,
                   buf: List[int],
                   src_reg: int,
                   src_dist: int,
                   stack: List[LevelStackEntry],
                   fill_stack: bool):
    w: int = chf.width
    h: int = chf.height

    i: int = 0
    x: int = 0
    y: int = 0
    c: Optional[CompactCell] = None
    if fill_stack:
        # Find cells revealed by the raised level
        stack.clear()
        for y in range(h):
            for x in range(w):
                c = chf.cells[x + y*w]
                if c is not None:
                    for i in range(c.index, c.index + c.count):
                        if chf.dist[i] >= level and buf[src_reg + i] == 0 and chf.areas[i] != RC_NULL_AREA:
                            stack.append(LevelStackEntry(x, y, i))
    else:
        # use cells in the input stack
        # mark all cells which already have a region
        for j in range(len(stack)):
            i = stack[j].index
            if buf[src_reg + i] != 0:
                stack[j].index = -1

    dirty_entries: List[DirtyEntry] = []
    iter: int = 0
    while len(stack) > 0:
        failed: int  = 0
        dirty_entries.clear()

        for j in range(len(stack)):
            x = stack[j].x
            y = stack[j].y
            i = stack[j].index
            if i < 0:
                failed += 1
                continue

            r: int = buf[src_reg + i]  # 2 bytes
            d2: int = 0xffff  # 65535, 2 bytes
            area: int = chf.areas[i]  # 1 byte
            s: Optional[CompactSpan] = chf.spans[i]
            if s is not None:
                for dir in range(4):
                    if get_con(s, dir) == RC_NOT_CONNECTED:
                        continue
                    ax: int = x + get_dir_offset_x(dir)
                    ay: int = y + get_dir_offset_y(dir)
                    c = chf.cells[ax + ay*w]
                    if c is not None:
                        ai: int = c.index + get_con(s, dir)
                        if chf.areas[ai] != area:
                            continue
                        if (buf[src_reg + ai] > 0) and ((buf[src_reg + ai] & RC_BORDER_REG) == 0):
                            if buf[src_dist + ai] + 2 < d2:
                                r = buf[src_reg + ai]
                                d2 = buf[src_dist + ai] + 2
            if r != 0:
                stack[j].index = -1  # mark as used
                dirty_entries.append(DirtyEntry(i, r, d2))
            else:
                failed += 1
        # Copy entries that differ between src and dst to keep them in sync
        for i_for in range(len(dirty_entries)):
            idx: int = dirty_entries[i_for].index
            buf[src_reg + idx] = dirty_entries[i_for].region
            buf[src_dist + idx] = dirty_entries[i_for].distance2

        if failed == len(stack):
            break

        if level > 0:
            iter += 1
            if iter >= max_iter:
                break

def flood_region(x: int,
                 y: int,
                 i: int,
                 level: int,  # 2 bytes
                 r: int,  # 2 bytes
                 chf: CompactHeightfield,
                 buf: List[int],
                 src_reg: int,
                 src_dist: int,
                 stack: List[LevelStackEntry]) -> bool:
    w: int = chf.width
    area: int = chf.areas[i]  # 1 byte

    # Flood fill mark region
    stack.clear()
    stack.append(LevelStackEntry(x, y, i))
    buf[src_reg + i] = r
    buf[src_dist + i] = 0

    lev: int = level - 2 if level >= 2 else 0  # 1 byte
    count: int = 0

    c1: Optional[CompactCell] = None
    c2: Optional[CompactCell] = None
    ax: int = 0
    ay: int = 0
    ai: int = 0
    while len(stack) > 0:
        back: LevelStackEntry = stack[-1]
        cx: int = back.x
        cy: int = back.y
        ci: int = back.index
        stack.pop(-1)

        cs: Optional[CompactSpan] = chf.spans[ci]

        # Check if any of the neighbours already have a valid region set
        ar: int = 0  # 1 byte
        if cs is not None:
            for dir in range(4):
                # 8 connected
                if get_con(cs, dir) != RC_NOT_CONNECTED:
                    ax = cx + get_dir_offset_x(dir)
                    ay = cy + get_dir_offset_y(dir)
                    c1 = chf.cells[ax + ay*w]
                    if c1 is not None:
                        ai = c1.index + get_con(cs, dir)
                        if chf.areas[ai] != area:
                            continue
                        nr: int = buf[src_reg + ai]  # 2 bytes
                        if nr & RC_BORDER_REG:  # Do not take borders into account
                            continue
                        if nr != 0 and nr != r:
                            ar = nr
                            break

                        asp: Optional[CompactSpan] = chf.spans[ai]

                        dir2: int = (dir + 1) & 0x3
                        if asp is not None:
                            if get_con(asp, dir2) != RC_NOT_CONNECTED:
                                ax2: int = ax + get_dir_offset_x(dir2)
                                ay2: int = ay + get_dir_offset_y(dir2)
                                c2 = chf.cells[ax2 + ay2*w]
                                if c2 is not None:
                                    ai2: int = c2.index + get_con(asp, dir2)
                                    if chf.areas[ai2] != area:
                                        continue
                                    nr2: int = buf[src_reg + ai2]  # 2 bytes
                                    if nr2 != 0 and nr2 != r:
                                        ar = nr2
                                        break
            if ar != 0:
                buf[src_reg + ci] = 0
                continue

            count += 1

            # Expand neighbours
            for dir in range(4):
                if get_con(cs, dir) != RC_NOT_CONNECTED:
                    ax = cx + get_dir_offset_x(dir)
                    ay = cy + get_dir_offset_y(dir)
                    c1 = chf.cells[ax + ay*w]
                    if c1 is not None:
                        ai = c1.index + get_con(cs, dir)
                        if chf.areas[ai] != area:
                            continue
                        if chf.dist[ai] >= lev and buf[src_reg + ai] == 0:
                            buf[src_reg + ai] = r
                            buf[src_dist + ai] = 0
                            stack.append(LevelStackEntry(ax, ay, ai))

    return count > 0

def add_unique_floor_region(reg: Region,
                            n: int):
    for i in range(len(reg.floors)):
        if reg.floors[i] == n:
            return
    reg.floors.append(n)

def is_solid_edge(chf: CompactHeightfield,
                  buf: List[int],
                  src_reg: int,
                  x: int,
                  y: int,
                  i: int, dir: int) -> bool:
    s: Optional[CompactSpan] = chf.spans[i]
    r: int = 0  # 2 bytes
    if s is not None:
        if get_con(s, dir) != RC_NOT_CONNECTED:
            ax: int = x + get_dir_offset_x(dir)
            ay: int = y + get_dir_offset_y(dir)
            c: Optional[CompactCell] = chf.cells[ax + ay*chf.width]
            if c is not None:
                ai: int = c.index + get_con(s, dir)
                r = buf[src_reg + ai]
    if r == buf[src_reg + i]:
        return False
    return True

def walk_contour(x: int,
                y: int,
                i: int,
                dir: int,
                chf: CompactHeightfield,
                buf: List[int],
                src_reg: int,
                cont: List[int]):
    start_dir: int = dir
    starti: int = i

    ss: Optional[CompactSpan] = chf.spans[i]
    cur_reg: int = 0  # 2 bytes
    ax: int = 0
    ay: int = 0
    ai: int = 0
    c: Optional[CompactCell] = None
    if ss is not None:
        if get_con(ss, dir) != RC_NOT_CONNECTED:
            ax = x + get_dir_offset_x(dir)
            ay = y + get_dir_offset_y(dir)
            c = chf.cells[ax + ay*chf.width]
            if c is not None:
                ai = c.index + get_con(ss, dir)
                cur_reg = buf[src_reg + ai]
    cont.append(cur_reg)

    iter: int = 1
    while iter < 40000:
        s: Optional[CompactSpan] = chf.spans[i]
        if s is not None:
            if is_solid_edge(chf, buf, src_reg, x, y, i, dir):
                # Choose the edge corner
                r: int = 0  # 2 bytes
                if get_con(s, dir) != RC_NOT_CONNECTED:
                    ax = x + get_dir_offset_x(dir)
                    ay = y + get_dir_offset_y(dir)
                    c = chf.cells[ax + ay*chf.width]
                    if c is not None:
                        ai = c.index + get_con(s, dir)
                        r = buf[src_reg + ai]
                if r != cur_reg:
                    cur_reg = r
                    cont.append(cur_reg)
                dir = (dir + 1) & 0x3  # Rotate CW
            else:
                ni: int = -1
                nx: int = x + get_dir_offset_x(dir)
                ny: int = y + get_dir_offset_y(dir)
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
                dir = (dir + 3) & 0x3  # Rotate CCW

        if starti == i and start_dir == dir:
            break
        iter += 1
    # Remove adjacent duplicates
    if len(cont) > 1:
        j: int = 0
        while j < len(cont):
            nj: int = (j + 1) % len(cont)
            if cont[j] == cont[nj]:
                for k in range(j, len(cont) - 1):
                    cont[k] = cont[k + 1]
                cont.pop(-1)
            else:
                j += 1

def is_region_connected_to_border(reg: Region) -> bool:
    for i in range(len(reg.connections)):
        if reg.connections[i] == 0:
            return True
    return False

def can_merge_with_region(rega: Region, 
                          regb: Region) -> bool:
    if rega.area_type != regb.area_type:
        return False
    n: int = 0
    for i in range(len(rega.connections)):
        if rega.connections[i] == regb.id:
            n += 1
    if n > 1:
        return False
    for i in range(len(rega.floors)):
        if rega.floors[i] == regb.id:
            return False
    return True

def remove_adjacent_neighbours(reg: Region):
    i: int = 0
    while i < len(reg.connections) and len(reg.connections) > 1:
        ni: int = (i+1) % len(reg.connections)
        if reg.connections[i] == reg.connections[ni]:
            for j in range(1, len(reg.connections) - 1):
                reg.connections[j] = reg.connections[j + 1]
            reg.connections.pop(-1)
        else:
            i += 1

def merge_regions(rega: Region,
                  regb: Region) -> bool:
    aid: int = rega.id  # 2 bytes
    bid: int = regb.id  # 2 bytes

    # Duplicate current neighbourhood
    acon: List[int] = [0] * len(rega.connections)
    for i in range(len(rega.connections)):
        acon[i] = rega.connections[i]
    bcon: List[int] = regb.connections

    # Find insertion point on A
    insa: int = -1
    for i in range(len(acon)):
        if acon[i] == bid:
            insa = i
            break
    if insa == -1:
        return False

    # Find insertion point on B
    insb: int = -1
    for i in range(len(bcon)):
        if bcon[i] == aid:
            insb = i
            break
    if insb == -1:
        return False

    # Merge neighbours
    rega.connections.clear()
    ni: int = len(acon)
    for i in range(ni - 1):
        rega.connections.append(acon[(insa + 1 + i) % ni])
    ni = len(bcon)
    for i in range(ni - 1):
        rega.connections.append(bcon[(insb + 1 + i) % ni])

    remove_adjacent_neighbours(rega)

    for j in range(len(regb.floors)):
        add_unique_floor_region(rega, regb.floors[j])
    rega.span_count += regb.span_count
    regb.span_count = 0
    regb.connections.clear()

    return True

def replace_neighbour(reg: Region,
                      old_id: int,  # 2 bytes
                      new_id: int):  # 2 bytes
    nei_changed: bool = False
    for i in range(len(reg.connections)):
        if reg.connections[i] == old_id:
            reg.connections[i] = new_id
            nei_changed = True
    for i in range(len(reg.floors)):
        if reg.floors[i] == old_id:
            reg.floors[i] = new_id
    if nei_changed:
        remove_adjacent_neighbours(reg)

def merge_and_filter_regions(min_region_area: int,
                             merge_region_size: int,
                             max_region_id: int,  # 2 bytes, changed in the function, so, we should also return it
                             chf: CompactHeightfield,
                             buf: List[int],
                             src_reg: int,
                             overlaps: List[int]) -> Tuple[bool, int]:
    w: int = chf.width
    h: int = chf.height

    nreg: int = max_region_id + 1
    regions: List[Region] = []

    # Construct regions
    for i in range(nreg):
        regions.append(Region(i))

    # Find edge of a region and find connections around the contour
    for y in range(h):
        for x in range(w):
            c: Optional[CompactCell] = chf.cells[x + y*w]
            if c is not None:
                for i in range(c.index, c.index + c.count):
                    r: int = buf[src_reg + i]  # 2 bytes
                    if r == 0 or r >= nreg:
                        continue
                    reg: Region = regions[r]
                    reg.span_count += 1

                    # Update floors
                    for j in range(c.index, c.index + c.count):
                        if i == j:
                            continue
                        floor_id: int = buf[src_reg + j]  # 2 bytes
                        if floor_id == 0 or floor_id >= nreg:
                            continue
                        if floor_id == r:
                            reg.overlap = True
                        add_unique_floor_region(reg, floor_id)

                    # Have found contour
                    if len(reg.connections) > 0:
                        continue

                    reg.area_type = chf.areas[i]

                    # Check if this cell is next to a border
                    ndir: int = -1
                    for dir in range(4):
                        if is_solid_edge(chf, buf, src_reg, x, y, i, dir):
                            ndir = dir
                            break

                    if ndir != -1:
                        # The cell is at border
                        # Walk around the contour to find all the neighbours
                        walk_contour(x, y, i, ndir, chf, buf, src_reg, reg.connections)

    # Remove too small regions
    stack: List[int] = [0] * 32  # <-- does not need create it nonempty
    trace: List[int] = [0] * 32
    for i in range(nreg):
        reg1: Region = regions[i]
        if reg1.id == 0 or (reg1.id & RC_BORDER_REG):
            continue
        if reg1.span_count == 0:
            continue
        if reg1.visited:
            continue

        # Count the total size of all the connected regions
        # Also keep track of the regions connects to a tile border
        connects_to_border: bool = False
        span_count: int = 0
        stack.clear()
        trace.clear()

        reg1.visited = True
        stack.append(i)

        while len(stack) > 0:
            ri: int = stack.pop(-1)
            creg: Region = regions[ri]
            span_count += creg.span_count
            trace.append(ri)

            for j in range(len(creg.connections)):
                if creg.connections[j] & RC_BORDER_REG:
                    connects_to_border = True
                    continue
                neireg: Region = regions[creg.connections[j]]
                if neireg.visited:
                    continue
                if neireg.id == 0 or (neireg.id & RC_BORDER_REG):
                    continue
                # Visit
                stack.append(neireg.id)
                neireg.visited = True
        # If the accumulated regions size is too small, remove it
        # Do not remove areas which connect to tile borders
        # as their size cannot be estimated correctly and removing them
        # can potentially remove necessary areas
        if span_count < min_region_area and not connects_to_border:
            # Kill all visited regions
            for j in range(len(trace)):
                regions[trace[j]].span_count = 0
                regions[trace[j]].id = 0

    # Merge too small regions to neighbour regions
    merge_count: int = 0
    is_do: bool = True
    old_id: int = 0
    while is_do:
        merge_count = 0
        for i in range(nreg):
            reg2: Region = regions[i]
            if reg2.id == 0 or (reg2.id & RC_BORDER_REG):
                continue
            if reg2.overlap:
                continue
            if reg2.span_count == 0:
                continue

            # Check to see if the region should be merged
            if reg2.span_count > merge_region_size and is_region_connected_to_border(reg2):
                continue

            # Small region with more than 1 connection
            # Or region which is not connected to a border at all
            # Find smallest neighbour region that connects to this one
            smallest: int = 0xfffffff  # 268 435 455
            merge_id: int = reg2.id  # 2 bytes
            for j in range(len(reg2.connections)):
                if reg2.connections[j] & RC_BORDER_REG:
                    continue
                mreg: Region = regions[reg2.connections[j]]
                if mreg.id == 0 or (mreg.id & RC_BORDER_REG) or mreg.overlap:
                    continue
                if mreg.span_count < smallest and can_merge_with_region(reg2, mreg) and can_merge_with_region(mreg, reg2):
                    smallest = mreg.span_count
                    merge_id = mreg.id
            # Found new id
            if merge_id != reg2.id:
                old_id = reg2.id  # 2 butes
                target: Region = regions[merge_id]

                # Merge neighbours
                if merge_regions(target, reg2):
                    # Fixup regions pointing to current region
                    for j in range(nreg):
                        if regions[j].id == 0 or (regions[j].id & RC_BORDER_REG):
                            continue
                        # If another region was already merged into current region
                        # change the nid of the previous region too
                        if regions[j].id == old_id:
                            regions[j].id = merge_id
                        # Replace the current region with the new one if the
                        # current regions is neighbour
                        replace_neighbour(regions[j], old_id, merge_id)
                    merge_count += 1

        is_do = merge_count > 0
    # Compress region Ids
    for i in range(nreg):
        regions[i].remap = False
        if regions[i].id == 0:
            continue
        if regions[i].id & RC_BORDER_REG:
            continue
        regions[i].remap = True

    reg_id_gen: int = 0  # 2 bytes
    for i in range(nreg):
        if not regions[i].remap:
            continue
        old_id = regions[i].id  # 2 bytes
        reg_id_gen += 1
        new_id: int = reg_id_gen
        for j in range(i, nreg):
            if regions[j].id == old_id:
                regions[j].id = new_id
                regions[j].remap = False
    max_region_id = reg_id_gen

    # Remap regions
    for i in range(chf.span_count):
        if buf[src_reg + i] & RC_BORDER_REG == 0:
            buf[src_reg + i] = regions[buf[src_reg + i]].id

    # Return regions that we found to be overlapping
    for i in range(nreg):
        if regions[i].overlap:
            overlaps.append(regions[i].id)

    return (True, max_region_id)

def build_regions(chf: CompactHeightfield, 
                  border_size: int, 
                  min_region_area: int,
                  merge_region_area: int) -> bool:
    w: int = chf.width
    h: int = chf.height

    buf: List[int] = [0] * (chf.span_count * 2)  # 2 bytes per element

    LOG_NB_STACKS: int = 3
    NB_STACKS: int = 1 << LOG_NB_STACKS  # in fact = 8
    lvl_stacks: List[List[LevelStackEntry]] = [[LevelStackEntry() for j in range(256)] for i in range(NB_STACKS)]
    stack: List[LevelStackEntry] = [LevelStackEntry() for i in range(256)]

    src_reg: int = 0  # first pointer to the buffer
    src_dist: int = chf.span_count  # second pointer to the buffer

    region_id: int = 1  # 2 bytes
    level: int = (chf.max_distance + 1) & ~1  # equal to the first argument

    expand_iters: int = 8

    if border_size > 0:
        # Make sure border will not overflow
        bw: int = min(w, border_size)
        bh: int = min(h, border_size)

        # Paint regions
        paint_rect_region(0, bw, 0, h, region_id | RC_BORDER_REG, chf, buf, src_reg)
        region_id += 1
        paint_rect_region(w - bw, w, 0, h, region_id | RC_BORDER_REG, chf, buf, src_reg)
        region_id += 1
        paint_rect_region(0, w, 0, bh, region_id | RC_BORDER_REG, chf, buf, src_reg)
        region_id += 1
        paint_rect_region(0, w, h - bh, h, region_id | RC_BORDER_REG, chf, buf, src_reg)
        region_id += 1

    chf.border_size = border_size

    s_id: int = -1
    while level > 0:
        level = level - 2 if level >= 2 else 0
        s_id = (s_id+1) & (NB_STACKS-1)

        if s_id == 0:
            sort_cells_by_level(level, chf, buf, src_reg, NB_STACKS, lvl_stacks, 1)
        else:
            append_stacks(lvl_stacks[s_id - 1], lvl_stacks[s_id], buf, src_reg)  # copy left overs from last level

        # Expand current regions until no empty connected cells found
        expand_regions(expand_iters, level, chf, buf, src_reg, src_dist, lvl_stacks[s_id], False)
        # Mark new regions with IDs
        for j in range(len(lvl_stacks[s_id])):
            current: LevelStackEntry = lvl_stacks[s_id][j]
            x: int = current.x
            y: int = current.y
            i = current.index
            if i >= 0 and buf[src_reg + i] == 0:
                if flood_region(x, y, i, level, region_id, chf, buf, src_reg, src_dist, stack):
                    if region_id == 0xFFFF:  # 65535
                        print("[Navmesh Baker] build_regions: Region ID overflow")
                        return False
                    region_id += 1

    # Expand current regions until no empty connected cells found
    expand_regions(expand_iters * 8, 0, chf, buf, src_reg, src_dist, stack, True)

    overlaps: List[int] = []
    chf.max_regions = region_id
    mafr_result = merge_and_filter_regions(min_region_area, merge_region_area, chf.max_regions, chf, buf, src_reg, overlaps)
    chf.max_regions = mafr_result[1]
    if not mafr_result[0]:
        return False

    # If overlapping regions were found during merging, split those regions
    if len(overlaps) > 0:
        print("[Navmesh Baker] build_regions: " + str(len(overlaps)) + " overlapping regions")

    # Write the result out
    for i in range(chf.span_count):
        s: Optional[CompactSpan] = chf.spans[i]
        if s is not None:
            s.reg = buf[src_reg + i]

    return True
