from typing import Tuple, List
import math
from pathfinder.navmesh_baker.rc_classes import CompactSpan

# return w and h
def calc_grid_size(bmin: Tuple[float, float, float],
                   bmax: Tuple[float, float, float],
                   cs: float) -> Tuple[int, int]:
    return int((bmax[0] - bmin[0]) / cs + 0.5), int((bmax[2] - bmin[2]) / cs + 0.5)


def v_sub(dest: List[float], v1: List[float], v2: List[float]):
    dest[0] = v1[0] - v2[0]
    dest[1] = v1[1] - v2[1]
    dest[2] = v1[2] - v2[2]


def v_cross(dest: List[float], v1: List[float], v2: List[float]):
    dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
    dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
    dest[2] = v1[0]*v2[1] - v1[1]*v2[0]


def v_normalize(v: List[float]):
    d: float = 1.0 / math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    v[0] = v[0] * d
    v[1] = v[1] * d
    v[2] = v[2] * d


def v_copy(dest: List[float], v: List[float], dest_shift: int = 0, src_shift: int = 0):
    dest[0 + dest_shift] = v[0 + src_shift]
    dest[1 + dest_shift] = v[1 + src_shift]
    dest[2 + dest_shift] = v[2 + src_shift]


def v_min(mn: List[float], v: List[float]):
    mn[0] = min(mn[0], v[0])
    mn[1] = min(mn[1], v[1])
    mn[2] = min(mn[2], v[2])


def v_max(mx: List[float], v: List[float]):
    mx[0] = max(mx[0], v[0])
    mx[1] = max(mx[1], v[1])
    mx[2] = max(mx[2], v[2])

def v_equal(a: int,
            b: int,
            array: List[int]) -> bool:
    return array[a] == array[b] and array[a + 2] == array[b + 2]

def calc_tri_normal(v0: List[float],
                    v1: List[float],
                    v2: List[float],
                    norm: List[float]):  # norm is output
    e0 = [0.0, 0.0, 0.0]
    e1 = [0.0, 0.0, 0.0]
    v_sub(e0, v1, v0)
    v_sub(e1, v2, v0)
    v_cross(norm, e0, e1)
    v_normalize(norm)


def get_dir_offset_x(dir: int):
    offset = [-1, 0, 1, 0]
    return offset[dir]


def get_dir_offset_y(dir: int):
    offset = [0, 1, 0, -1]
    return offset[dir]


def get_dir_offset(x: int, y: int):
    dirs = [3, 0, -1, 2, 1]
    return dirs[((y+1)<<1)+x]


def overlap_bounds(a_min: Tuple[float, float, float], a_max: Tuple[float, float, float], 
                   b_min: List[float], b_max: List[float]) -> bool:
    overlap = True
    if a_min[0] > b_max[0] or a_max[0] < b_min[0]:
        overlap = False
    if a_min[1] > b_max[1] or a_max[1] < b_min[1]:
        overlap = False
    if a_min[2] > b_max[2] or a_max[2] < b_min[2]:
        overlap = False
    return overlap


def clamp(v, mn, mx):
    if v < mn:
        return mn
    elif v > mx:
        return mx
    else:
        return v

# Sets the neighbor connection data for the specified direction
def set_con(s: CompactSpan, dir: int, i: int):
    shift: int = dir * 6
    con = s.con
    s.con = (con & ~(0x3f << shift)) | ((i & 0x3f) << shift)


# Gets neighbor connection data for the specified direction
def get_con(s: CompactSpan, dir: int) -> int:
    shift: int = dir * 6
    return (s.con >> shift) & 0x3f

def next(i: int,
         n: int) -> int:
    if i + 1 < n:
        return i + 1
    else:
        return 0

def prev(i: int,
         n: int) -> int:
    if i - 1 >= 0:
        return i - 1
    else:
        return n - 1
