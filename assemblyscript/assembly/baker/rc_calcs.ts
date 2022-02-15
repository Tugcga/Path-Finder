import { CompactSpan } from "./rc_classes";
import { List } from "../common/list";

type float = f64;
type int = i32;

@inline
export function calc_grid_size(bmin: StaticArray<float>,
                               bmax: StaticArray<float>,
                               cs: float): StaticArray<int>{
    var to_return = new StaticArray<int>(2);
    to_return[0] = i32((bmax[0] - bmin[0]) / cs + 0.5);
    to_return[1] = i32((bmax[2] - bmin[2]) / cs + 0.5);
    return to_return;
}

@inline
export function v_sub(dest: StaticArray<float>, v1: StaticArray<float>, v2: StaticArray<float>): void{
    dest[0] = v1[0] - v2[0];
    dest[1] = v1[1] - v2[1];
    dest[2] = v1[2] - v2[2];
}

@inline
export function v_cross(dest: StaticArray<float>, v1: StaticArray<float>, v2: StaticArray<float>): void{
    dest[0] = v1[1]*v2[2] - v1[2]*v2[1];
    dest[1] = v1[2]*v2[0] - v1[0]*v2[2];
    dest[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

@inline
export function v_normalize(v: StaticArray<float>): void{
    let d: float = 1.0 / Math.sqrt(v[0]**2 + v[1]**2 + v[2]**2);
    v[0] = v[0] * d;
    v[1] = v[1] * d;
    v[2] = v[2] * d;
}

@inline
export function v_copy(dest: StaticArray<float>, v: StaticArray<float>, dest_shift: int = 0, src_shift: int = 0): void{
    dest[0 + dest_shift] = v[0 + src_shift];
    dest[1 + dest_shift] = v[1 + src_shift];
    dest[2 + dest_shift] = v[2 + src_shift];
}

@inline
export function v_min(mn: StaticArray<float>, v: StaticArray<float>): void{
    mn[0] = Math.min(mn[0], v[0]);
    mn[1] = Math.min(mn[1], v[1]);
    mn[2] = Math.min(mn[2], v[2]);
}

@inline
export function v_max(mx: StaticArray<float>, v: StaticArray<float>): void{
    mx[0] = Math.max(mx[0], v[0]);
    mx[1] = Math.max(mx[1], v[1]);
    mx[2] = Math.max(mx[2], v[2]);
}

@inline
export function v_equal_list(a: int,
                             b: int,
                             array: List<int>): bool{
    return array[a] == array[b] && array[a + 2] == array[b + 2];
}

@inline
export function v_equal_static(a: int,
                               b: int,
                               array: StaticArray<int>): bool{
    return array[a] == array[b] && array[a + 2] == array[b + 2];
}

@inline
export function calc_tri_normal(v0: StaticArray<float>,
                                v1: StaticArray<float>,
                                v2: StaticArray<float>,
                                norm: StaticArray<float>): void{  // norm is output
    let e0: StaticArray<float> = new StaticArray<float>(3);
    let e1: StaticArray<float> = new StaticArray<float>(3);
    v_sub(e0, v1, v0);
    v_sub(e1, v2, v0);
    v_cross(norm, e0, e1);
    v_normalize(norm);
}

const offset_x = [-1, 0, 1, 0];

@inline
export function get_dir_offset_x(dir: int): int{
    return offset_x[dir];
}

const offset_y = [0, 1, 0, -1];

@inline
export function get_dir_offset_y(dir: int): int{
    return offset_y[dir];
}

const dirs = [3, 0, -1, 2, 1];

@inline
export function get_dir_offset(x: int, y: int): int{
    return dirs[((y+1)<<1)+x];
}

@inline
export function overlap_bounds(a_min: StaticArray<float>, 
                               a_max: StaticArray<float>, 
                               b_min: StaticArray<float>,
                               b_max: StaticArray<float>): bool{
    var overlap = true;
    if(a_min[0] > b_max[0] || a_max[0] < b_min[0]){
        overlap = false;
    }
    if(a_min[1] > b_max[1] || a_max[1] < b_min[1]){
        overlap = false;
    }
    if(a_min[2] > b_max[2] || a_max[2] < b_min[2]){
        overlap = false;
    }
    return overlap;
}

@inline
export function clamp(v: float, mn: float, mx: float): float{
    if(v < mn){
        return mn;
    }
    else if(v > mx){
        return mx;
    }
    else{
        return v;
    }
}

@inline
export function clamp_int(v: int, mn: int, mx: int): int{
    if(v < mn){
        return mn;
    }
    else if(v > mx){
        return mx;
    }
    else{
        return v;
    }
}

@inline
export function set_con(s: CompactSpan, dir: int, i: int): void{
    let shift: int = dir * 6;
    let con = s.con;
    s.con = (con & ~(0x3f << shift)) | ((i & 0x3f) << shift);
}

@inline
export function get_con(s: CompactSpan, dir: int): int{
    let shift: int = dir * 6;
    return (s.con >> shift) & 0x3f;
}

@inline
export function next(i: int,
                     n: int): int{
    if(i + 1 < n){
        return i + 1;
    }
    else{
        return 0;
    }
}

@inline
export function prev(i: int,
                     n: int): int{
    if(i - 1 >= 0){
        return i - 1;
    }
    else{
        return n - 1;
    }
}

@inline
export function slice(array: StaticArray<float>, start: int, end: int): StaticArray<float>{
    let to_return = new StaticArray<float>(end - start);
    for(let i = 0; i < end - start; i++){
        to_return[i] = array[start + i];
    }
    return to_return;
}

@inline
export function build_int_pair(a: int, b: int): StaticArray<int>{
    let to_return = new StaticArray<int>(2);
    to_return[0] = a;
    to_return[1] = b;
    return to_return;
}

@inline
export function build_int_triple(a: int, b: int, c: int): StaticArray<int>{
    let to_return = new StaticArray<int>(3);
    to_return[0] = a;
    to_return[1] = b;
    to_return[2] = c;
    return to_return;
}