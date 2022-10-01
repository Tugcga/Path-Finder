import { CompactHeightfield, ContourSet, Contour, CompactCell, CompactSpan, ContourRegion, ContourHole, PotentialDiagonal } from "./rc_classes";
import { RC_CONTOUR_TESS_WALL_EDGES, RC_BORDER_REG, RC_NOT_CONNECTED, RC_BORDER_VERTEX, RC_AREA_BORDER, RC_CONTOUR_REG_MASK, RC_CONTOUR_TESS_AREA_EDGES } from "./rc_constants";
import { get_con, get_dir_offset_x, get_dir_offset_y, v_equal_list, next, prev } from "./rc_calcs";
import { log_message } from "../common/utilities";
import { List } from "../common/list";


type int = i32;
type float = f64;

function get_corner_height(x: int,
                           y: int,
                           i: int,
                           dir: int,
                           chf: CompactHeightfield,
                           is_border_vertex: bool): StaticArray<int>{
    let s = chf.spans[i];
    let ch: int = 0;
    let ax: int = 0;
    let ay: int = 0;
    let ai: int = 0;
    let asp: CompactSpan | null = null;
    let ax2: int = 0;
    let ay2: int = 0;
    let ai2: int = 0;
    let as2: CompactSpan | null = null;
    if(s){
        ch = s.y;
        const dirp: int = (dir+1) & 0x3;
        
        let regs: StaticArray<int> = new StaticArray<int>(4);
        regs[0] = s.reg | (chf.areas[i] << 16);
        
        if(get_con(s, dir) != RC_NOT_CONNECTED){
            ax = x + get_dir_offset_x(dir);
            ay = y + get_dir_offset_y(dir);
            let c = chf.cells[ax+ay*chf.width];
            if(c){
                ai = c.index + get_con(s, dir);
                asp = chf.spans[ai];
                if(asp){
                    ch = <i32>Math.max(ch, asp.y);
                    regs[1] = asp.reg | (chf.areas[ai] << 16);
                    if(get_con(asp, dirp) != RC_NOT_CONNECTED){
                        ax2 = ax + get_dir_offset_x(dirp);
                        ay2 = ay + get_dir_offset_y(dirp);
                        let c2 = chf.cells[ax2+ay2*chf.width];
                        if(c2){
                            ai2 = c2.index + get_con(asp, dirp);
                            as2 = chf.spans[ai2];
                            if(as2){
                                ch = <i32>Math.max(ch, as2.y);
                                regs[2] = as2.reg | (chf.areas[ai2] << 16);
                            }
                        }
                    }
                }
            }
        }

        if(get_con(s, dirp) != RC_NOT_CONNECTED){
            ax = x + get_dir_offset_x(dirp);
            ay = y + get_dir_offset_y(dirp);
            let cc = chf.cells[ax+ay*chf.width];
            if(cc){
                ai = cc.index + get_con(s, dirp);
                asp = chf.spans[ai];
                if(asp){
                    ch = <i32>Math.max(ch, asp.y);
                    regs[3] = asp.reg | (chf.areas[ai] << 16);
                    if(get_con(asp, dir) != RC_NOT_CONNECTED){
                        ax2 = ax + get_dir_offset_x(dir);
                        ay2 = ay + get_dir_offset_y(dir);
                        let ccc = chf.cells[ax2+ay2*chf.width];
                        if(ccc){
                            ai2 = ccc.index + get_con(asp, dir);
                            as2 = chf.spans[ai2];
                            if(as2){
                                ch = <i32>Math.max(ch, as2.y);
                                regs[2] = as2.reg | (chf.areas[ai2] << 16);
                            }
                        }
                    }
                }
            }
        }

        for(let j = 0; j < 4; j++){
            const av: int = j;
            const bv: int = (j+1) & 0x3;
            const cv: int = (j+2) & 0x3;
            const dv: int = (j+3) & 0x3;
            
            const two_same_exts: bool = (regs[av] & regs[bv] & RC_BORDER_REG) != 0 && (regs[av] == regs[bv]);
            const two_ints: bool = ((regs[cv] | regs[dv]) & RC_BORDER_REG) == 0;
            const ints_same_area: bool = (regs[cv]>>16) == (regs[dv]>>16);
            const no_zeros: bool = (regs[av] != 0) && (regs[bv] != 0) && (regs[cv] != 0) && (regs[dv] != 0);
            if(two_same_exts && two_ints && ints_same_area && no_zeros){
                is_border_vertex = true;
                break;
            }
        }
    }
    
    let to_return = new StaticArray<int>(2);
    to_return[0] = ch;
    to_return[1] = is_border_vertex ? 1 : 0;
    return to_return;
}

function walk_contour(x: int,
                      y: int,
                      i: int,
                      chf: CompactHeightfield,
                      flags: StaticArray<int>,
                      points: List<int>): void{
    let dir: int = 0;  // 1 byte
    while((flags[i] & (1 << dir)) == 0){
        dir += 1;
    }

    let start_dir: int = dir;  // 1 byte
    let starti: int = i;

    let area: int = chf.areas[i];  // 1 byte

    let iter: int = 0;
    let s: CompactSpan | null = null;
    while(iter < 40000){
        iter += 1;
        if(flags[i] & (1 << dir)){
            let is_border_vertex: bool = false;
            let is_area_border: bool = false;
            let px: int = x;
            let get_corner_height_result = get_corner_height(x, y, i, dir, chf, is_border_vertex);
            let py = get_corner_height_result[0];
            is_border_vertex = get_corner_height_result[1] == 1;
            let pz: int = y;
            if(dir == 0){
                pz += 1;
            }
            else if(dir == 1){
                px += 1;
                pz += 1;
            }
            else if(dir == 2){
                px += 1;
            }
            let r: int = 0;
            s = chf.spans[i];
            if(s){
                if(get_con(s, dir) != RC_NOT_CONNECTED){
                    const ax: int = x + get_dir_offset_x(dir);
                    const ay: int = y + get_dir_offset_y(dir);
                    let c: CompactCell | null = chf.cells[ax + ay*chf.width];
                    if(c){
                        const ai: int = c.index + get_con(s, dir);
                        let ss: CompactSpan | null = chf.spans[ai];
                        if(ss){
                            r = ss.reg;
                        }
                        if(area != chf.areas[ai]){
                            is_area_border = true;
                        }
                    }
                    if(is_border_vertex){
                        r |= RC_BORDER_VERTEX;
                    }
                    if(is_area_border){
                        r |= RC_AREA_BORDER;
                    }
                    points.push(px);
                    points.push(py);
                    points.push(pz);
                    points.push(r);

                    flags[i] &= ~(1 << dir);
                    dir = (dir + 1) & 0x3;
                }
            }
        }
        else{
            let ni: int = -1;
            const nx: int = x + get_dir_offset_x(dir);
            const ny: int = y + get_dir_offset_y(dir);
            s = chf.spans[i];
            if(s){
                if(get_con(s, dir) != RC_NOT_CONNECTED){
                    let nc: CompactCell | null = chf.cells[nx + ny*chf.width];
                    if(nc){
                        ni = nc.index + get_con(s, dir);
                    }
                }
            }
            if(ni == -1){
                return;
            }
            x = nx;
            y = ny;
            i = ni;
            dir = (dir + 3) & 0x3;
        }

        if(starti == i && start_dir == dir){
            break;
        }
    }
}

@inline
function distance_pt_seg(x: int,
                         z: int,
                         px: int,
                         pz: int,
                         qx: int,
                         qz: int): float{
    const pqx: float = <f64>(qx - px);
    const pqz: float = <f64>(qz - pz);
    let dx: float = <f64>(x - px);
    let dz: float = <f64>(z - pz);
    const d: float = pqx*pqx + pqz*pqz;
    let t: float = pqx*dx + pqz*dz;
    if(d > 0.0){
        t /= d;
    }
    if(t < 0.0){
        t = 0.0;
    }
    else if(t > 1.0){
        t = 1.0;
    }
    
    dx = <f64>(px) + t*pqx - <f64>(x);
    dz = <f64>(pz) + t*pqz - <f64>(z);
    
    return dx*dx + dz*dz;
}

function simplify_contour(points: List<int>,
                          simplified: List<int>,
                          max_error: float,
                          max_edge_len: int,
                          build_flags: int): void{
    let has_connections: bool = false;
    let i: int = 0;
    while(i < points.length){
        if((points[i + 3] & RC_CONTOUR_REG_MASK) != 0){
            has_connections = true;
            break;
        }
        i += 4;
    }

    let ii: int = 0;
    let ax: int = 0;
    let az: int = 0;
    let ai: int = 0;
    let bx: int = 0;
    let bz: int = 0;
    let bi: int = 0;
    
    let maxi: int = -1;
    const pn: int = points.length / 4;
    let ci: int = (ai+1) % pn;

    if(has_connections){
        const ni: int = points.length / 4;
        i = 0;
        while(i < ni){
            ii = (i + 1) % ni;
            const different_regs: bool = (points[i*4+3] & RC_CONTOUR_REG_MASK) != (points[ii*4+3] & RC_CONTOUR_REG_MASK);
            const area_borders: bool = (points[i*4+3] & RC_AREA_BORDER) != (points[ii*4+3] & RC_AREA_BORDER);
            if(different_regs || area_borders){
                simplified.push(points[i*4]);
                simplified.push(points[i*4 + 1]);
                simplified.push(points[i*4 + 2]);
                simplified.push(i);
            }
            i += 1
        }
    }

    if(simplified.length == 0){
        let llx: int = points[0];
        let lly: int = points[1];
        let llz: int = points[2];
        let lli: int = 0;
        let urx: int = points[0];
        let ury: int = points[1];
        let urz: int = points[2];
        let uri: int = 0;
        i = 0
        while(i < points.length){
            const x: int = points[i+0];
            const y: int = points[i+1];
            const z: int = points[i+2];
            if(x < llx || (x == llx && z < llz)){
                llx = x;
                lly = y;
                llz = z;
                lli = i / 4;
            }
            if(x > urx || (x == urx && z > urz)){
                urx = x;
                ury = y;
                urz = z;
                uri = i / 4;
            }
            i += 4;
        }
        simplified.push(llx);
        simplified.push(lly);
        simplified.push(llz);
        simplified.push(lli);
        
        simplified.push(urx);
        simplified.push(ury);
        simplified.push(urz);
        simplified.push(uri);
    }

    i = 0;
    let n: int = 0;
    while(i < simplified.length / 4){
        ii = (i+1) % (simplified.length / 4);
        
        ax = simplified[i*4+0];
        az = simplified[i*4+2];
        ai = simplified[i*4+3];

        bx = simplified[ii*4+0];
        bz = simplified[ii*4+2];
        bi = simplified[ii*4+3];

        let maxd: float = 0.0;
        maxi = -1;
        ci = 0;
        let cinc: int = 0;
        let endi: int = 0;

        if(bx > ax || (bx == ax && bz > az)){
            cinc = 1
            ci = (ai+cinc) % pn
            endi = bi
        }
        else{
            cinc = pn-1;
            ci = (bi+cinc) % pn;
            endi = ai;
            const temp1 = ax;
            ax = bx; bx = temp1;
            const temp2 = az;
            az = bz; bz = temp2;
        }

        if((points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0 || (points[ci*4+3] & RC_AREA_BORDER)){
            while(ci != endi){
                let d: float = distance_pt_seg(points[ci*4+0], points[ci*4+2], ax, az, bx, bz);
                if(d > maxd){
                    maxd = d;
                    maxi = ci;
                }
                ci = (ci+cinc) % pn;
            }
        }

        if(maxi != -1 && maxd > (max_error*max_error)){
            simplified.push(0); simplified.push(0); simplified.push(0); simplified.push(0);
            n = simplified.length / 4;
            for(let j = n - 1; j > i; j--){
                simplified[j*4+0] = simplified[(j-1)*4+0];
                simplified[j*4+1] = simplified[(j-1)*4+1];
                simplified[j*4+2] = simplified[(j-1)*4+2];
                simplified[j*4+3] = simplified[(j-1)*4+3];
            }
            simplified[(i+1)*4+0] = points[maxi*4+0];
            simplified[(i+1)*4+1] = points[maxi*4+1];
            simplified[(i+1)*4+2] = points[maxi*4+2];
            simplified[(i+1)*4+3] = maxi;
        }
        else{
            i += 1;
        }
    }

    if(max_edge_len > 0 && (build_flags & (RC_CONTOUR_TESS_WALL_EDGES | RC_CONTOUR_TESS_AREA_EDGES)) != 0){
        i = 0;
        while(i < simplified.length / 4){
            ii = (i+1) % (simplified.length / 4);
            
            ax = simplified[i*4+0];
            az = simplified[i*4+2];
            ai = simplified[i*4+3];
            
            bx = simplified[ii*4+0];
            bz = simplified[ii*4+2];
            bi = simplified[ii*4+3];
            
            maxi = -1;
            ci = (ai+1) % pn;
            
            let tess: bool = false;
            if((build_flags & RC_CONTOUR_TESS_WALL_EDGES) && (points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0){
                tess = true;
            }
            if((build_flags & RC_CONTOUR_TESS_AREA_EDGES) && (points[ci*4+3] & RC_AREA_BORDER)){
                tess = true;
            }
            
            if(tess){
                const dx: int = bx - ax;
                const dz: int = bz - az;
                if(dx*dx + dz*dz > max_edge_len*max_edge_len){
                    n = bi < ai ? (bi+pn - ai) : (bi - ai);
                    if(n > 1){
                        if(bx > ax || (bx == ax && bz > az)){
                            maxi = (ai + n / 2) % pn;
                        }
                        else{
                            maxi = (ai + (n+1) / 2) % pn;
                        }
                    }
                }
            }
            
            if(maxi != -1){
                simplified.push(0); simplified.push(0); simplified.push(0); simplified.push(0);
                n = simplified.length / 4;
                for(let j = n - 1; j > i; j--){
                    simplified[j*4+0] = simplified[(j-1)*4+0];
                    simplified[j*4+1] = simplified[(j-1)*4+1];
                    simplified[j*4+2] = simplified[(j-1)*4+2];
                    simplified[j*4+3] = simplified[(j-1)*4+3];
                }
                simplified[(i+1)*4+0] = points[maxi*4+0];
                simplified[(i+1)*4+1] = points[maxi*4+1];
                simplified[(i+1)*4+2] = points[maxi*4+2];
                simplified[(i+1)*4+3] = maxi;
            }
            else{
                i += 1;
            }
        }
    }
    
    i = 0;
    while(i < simplified.length / 4){
        ai = (simplified[i*4+3]+1) % pn;
        bi = simplified[i*4+3];
        simplified[i*4+3] = (points[ai*4+3] & (RC_CONTOUR_REG_MASK|RC_AREA_BORDER)) | (points[bi*4+3] & RC_BORDER_VERTEX);
        i += 1;
    }
}

function remove_degenerate_segments(simplified: List<int>): void{
    let npts: int = simplified.length / 4;
    let i: int = 0;
    while(i < npts){
        const ni: int = next(i, npts);
        if(v_equal_list(4*i, 4*ni, simplified)){
            for(let j = i; j < simplified.length / 4 - 1; j++){
                simplified[j*4+0] = simplified[(j+1)*4+0];
                simplified[j*4+1] = simplified[(j+1)*4+1];
                simplified[j*4+2] = simplified[(j+1)*4+2];
                simplified[j*4+3] = simplified[(j+1)*4+3];
            }
            simplified.pop_last();
            simplified.pop_last();
            simplified.pop_last();
            simplified.pop_last();
            npts -= 1;
        }
        i += 1;
    }
}

@inline
function calc_area_of_polygon_2d(verts: StaticArray<int>,
                                 nverts: int): int{
    let area: int = 0;
    let j: int = nverts - 1;
    for(let i = 0; i < nverts; i++){
        area += verts[4*i] * verts[4*j + 2] - verts[4*j] * verts[4*i + 2];
        j = i;
    }
    return (area + 1) / 2;
}

function find_left_most_vertex(contour: Contour): StaticArray<int>{
    let minx: int = contour.verts[0];
    let minz: int = contour.verts[2];
    let leftmost: int = 0;
    for(let i = 1; i <  contour.nverts; i++){
        const x: int = contour.verts[i*4];
        const z: int = contour.verts[i*4 + 2];
        if(x < minx || (x == minx && z < minz)){
            minx = x;
            minz = z;
            leftmost = i;
        }
    }
    let to_return = new StaticArray<int>(3);
    to_return[0] = minx;
    to_return[1] = minz;
    to_return[2] = leftmost;
    return to_return;
}

@inline
function compare_holes(a: ContourHole,
                       b: ContourHole): int{
    if(a.minx == b.minx){
        if(a.minz < b.minz){
            return -1;
        }
        if(a.minz > b.minz){
            return 1;
        }
    }
    else{
        if(a.minx < b.minx){
            return -1;
        }
        if(a.minx > b.minx){
            return 1;
        }
    }
    return 0;
}

@inline
function area2(array: StaticArray<int>,
               a: int,
               b: int,
               c: int): int{
    return (array[b] - array[a]) * (array[c + 2] - array[a + 2]) - (array[c] - array[a]) * (array[b + 2] - array[a + 2]);
}

@inline
function left_on(array: StaticArray<int>,
                 a: int,
                 b: int,
                 c: int): bool{
    return area2(array, a, b, c) <= 0;
}

@inline
function left(a: int,
              a_array: StaticArray<int>,
              b: int,
              b_array: StaticArray<int>,
              c: int,
              c_array: StaticArray<int>): bool{
    const v: int = (b_array[b]-a_array[a])*(c_array[c+2]-a_array[a+2]) - (c_array[c]-a_array[a])*(b_array[b+2]-a_array[a+2]);
    return v < 0;
}

@inline
function xorb(x: bool,
              y: bool): bool{
    return !((!x) == (!y));
}

@inline
function in_cone(i: int,
                 n: int,
                 verts: StaticArray<int>,
                 pj: int,
                 p_array: StaticArray<int>): bool{
    const pi: int = i * 4;
    const pi1: int = next(i, n) * 4;
    const pin1: int = prev(i, n) * 4;

    const v1: int = (p_array[pj]-verts[pi])*(verts[pin1+2]-verts[pi+2]) - (verts[pin1]-verts[pi])*(p_array[pj+2]-verts[pi+2]);
    const v2: int = (verts[pi]-p_array[pj])*(verts[pi1+2]-p_array[pj+2]) - (verts[pi1]-p_array[pj])*(verts[pi+2]-p_array[pj+2]);
    if(left_on(verts, pin1, pi, pi1)){
        return v1 < 0 && v2 < 0;
    }
    return !(v1 <= 0 && v2 <= 0);
}

@inline
function compare_diag_dist(a: PotentialDiagonal,
                           b: PotentialDiagonal): int{
    if(a.dist < b.dist){
        return -1;
    }
    if(a.dist > b.dist){
        return 1;
    }
    return 0;
}

@inline
function collinear(a: int,
                   a_array: StaticArray<int>,
                   b: int,
                   b_array: StaticArray<int>,
                   c: int,
                   c_array: StaticArray<int>): bool{
    const v: int = (b_array[b]-a_array[a])*(c_array[c+2]-a_array[a+2]) - (c_array[c]-a_array[a])*(b_array[b+2]-a_array[a+2]);
    return v == 0;
}

@inline
function intersect_prop(a: int,
                        a_array: StaticArray<int>,
                        b: int,
                        b_array: StaticArray<int>,
                        c: int,
                        c_array: StaticArray<int>,
                        d: int,
                        d_array: StaticArray<int>): bool{
    if(collinear(a, a_array, b, b_array, c, c_array) ||
       collinear(a, a_array, b, b_array, d, d_array) || 
       collinear(c, c_array, d, d_array, a, a_array) || 
       collinear(c, c_array, d, d_array, b, b_array)){
        return false;
    }
    return xorb(left(a, a_array, b, b_array, c, c_array), left(a, a_array, b, b_array, d, d_array)) && xorb(left(c, c_array, d, d_array, a, a_array), left(c, c_array, d, d_array, b, b_array));
}

@inline
function between(a: int,
                 a_array: StaticArray<int>,
                 b: int,
                 b_array: StaticArray<int>,
                 c: int,
                 c_array: StaticArray<int>): bool{
    if(!collinear(a, a_array, b, b_array, c, c_array)){
        return false;
    }
    if(a_array[a] != b_array[b]){
        return ((a_array[a] <= c_array[c]) && (c_array[c] <= b_array[b])) || ((a_array[a] >= c_array[c]) && (c_array[c] >= b_array[b]));
    }
    else{
        return ((a_array[a+2] <= c_array[c+2]) && (c_array[c+2] <= b_array[b+2])) || ((a_array[a+2] >= c_array[c+2]) && (c_array[c+2] >= b_array[b+2]));
    }
}

@inline
function intersect(a: int,
                   a_array: StaticArray<int>,
                   b: int,
                   b_array: StaticArray<int>,
                   c: int,
                   c_array: StaticArray<int>,
                   d: int,
                   d_array: StaticArray<int>): bool{
    if(intersect_prop(a, a_array, b, b_array, c, c_array, d, d_array)){
        return true;
    }
    else if(between(a, a_array, b, b_array, c, c_array) || 
            between(a, a_array, b, b_array, d, d_array) || 
            between(c, c_array, d, d_array, a, a_array) || 
            between(c, c_array, d, d_array, b, b_array)){
        return true;
    }
    else{
        return false;
    }
}

function intersect_seg_countour(d0: int,
                                d0_array: StaticArray<int>,
                                d1: int,
                                d1_array: StaticArray<int>,
                                i: int,
                                n: int,
                                verts: StaticArray<int>): bool{
    for(let k = 0; k < n; k++){
        const k1: int = next(k, n);
        if(i == k || i == k1){
            continue;
        }
        const p0: int = k*4;
        const p1: int = k1*4;
        if((d0_array[d0] == verts[p0] && d0_array[d0+2] == verts[p0+2]) ||
           (d1_array[d1] == verts[p0] && d1_array[d1+2] == verts[p0+2]) || 
           (d0_array[d0] == verts[p1] && d0_array[d0+2] == verts[p1+2]) || 
           (d1_array[d1] == verts[p1] && d1_array[d1+2] == verts[p1+2])){
            continue;
        }
        if(intersect(d0, d0_array, d1, d1_array, p0, verts, p1, verts)){
            return true;
        }
    }
    return false;
}

function merge_contours(ca: Contour,
                        cb: Contour,
                        ia: int,
                        ib: int): bool{
    const maxVerts: int = ca.nverts + cb.nverts + 2;
    let verts: StaticArray<int> = new StaticArray<int>(maxVerts*4);
    
    let nv: int = 0;
    let dst: int = 0;
    let src: int = 0;
    
    for(let i = 0; i < ca.nverts + 1; i++){
        dst = nv*4;
        src = ((ia+i)%ca.nverts)*4;
        verts[dst] = ca.verts[src];
        verts[dst + 1] = ca.verts[src + 1];
        verts[dst + 2] = ca.verts[src + 2];
        verts[dst + 3] = ca.verts[src + 3];
        nv += 1;
    }

    for(let i = 0; i < cb.nverts + 1; i++){
        dst = nv*4;
        src = ((ib+i)%cb.nverts)*4;
        verts[dst] = cb.verts[src];
        verts[dst + 1] = cb.verts[src + 1];
        verts[dst + 2] = cb.verts[src + 2];
        verts[dst + 3] = cb.verts[src + 3];
        nv += 1;
    }

    ca.verts = verts;
    ca.nverts = nv;

    cb.verts = new StaticArray<int>(0);
    cb.nverts = 0
    
    return true;
}

function merge_region_holes(region: ContourRegion, holes: StaticArray<ContourHole>): void{
    let contour: Contour | null = null;
    for(let i = 0; i < region.nholes; i++){
        contour = holes[region.holes_index + i].contour;
        if(contour){
            let find_result = find_left_most_vertex(contour);
            holes[region.holes_index + i].minx = find_result[0];
            holes[region.holes_index + i].minz = find_result[1];
            holes[region.holes_index + i].leftmost = find_result[2];
        }
    }
    let sorted_holes: StaticArray<ContourHole> = holes.slice<StaticArray<ContourHole>>(region.holes_index, region.holes_index + region.nholes);
    sorted_holes.sort(compare_holes);

    for(let i = 0; i < region.nholes; i++){
        holes[region.holes_index + i] = sorted_holes[i];
    }

    let max_verts: int = 0;
    let reg_outline: Contour | null = region.outline;
    if(reg_outline){
         max_verts = reg_outline.nverts;
        for(let i = 0; i < region.nholes; i++){
            contour = holes[region.holes_index + i].contour;
            if(contour){
                max_verts += contour.nverts;
            }
        }
    }

    let diags: StaticArray<PotentialDiagonal> = new StaticArray<PotentialDiagonal>(max_verts);
    for(let i = 0; i < max_verts; i++){
        diags[i] = new PotentialDiagonal();
    }

    let outline: Contour | null = region.outline;
    if(outline){
        for(let i = 0; i < region.nholes; i++){
            let hole: Contour | null = holes[region.holes_index].contour;
            
            let index: int = -1;
            let bestVertex: int = holes[region.holes_index + i].leftmost;
            if(hole){
                for(let iter = 0; iter < hole.nverts; iter++){
                    let ndiags: int = 0;
                    const corner: int = bestVertex*4;
                    for(let j = 0; j < outline.nverts; j++){
                        if(in_cone(j, outline.nverts, outline.verts, corner, hole.verts)){
                            const dx: int = outline.verts[j*4+0] - hole.verts[corner];
                            const dz: int = outline.verts[j*4+2] - hole.verts[corner + 2];
                            diags[ndiags].vert = j;
                            diags[ndiags].dist = dx*dx + dz*dz;
                            ndiags += 1;
                        }
                    }
                    let sorted_diags: StaticArray<PotentialDiagonal> = diags.slice<StaticArray<PotentialDiagonal>>(0, ndiags);
                    sorted_diags.sort(compare_diag_dist);
                    for(let m = 0; m < ndiags; m++){
                        diags[m] = sorted_diags[m];
                    }
                    
                    index = -1;
                    for(let j = 0; j < ndiags; j++){
                        const pt: int = diags[j].vert*4;
                        let intersect: bool = intersect_seg_countour(pt, outline.verts, corner, hole.verts, diags[i].vert, outline.nverts, outline.verts);
                        
                        let k: int = i;
                        while(k < region.nholes && !intersect){
                            contour = holes[region.holes_index + k].contour;
                            if(contour){
                                intersect |= intersect_seg_countour(pt, outline.verts, corner, hole.verts, -1, contour.nverts, contour.verts);
                            }
                            k += 1;
                        }
                        if(!intersect){
                            index = diags[j].vert;
                            break;
                        }
                    }
                    if(index != -1){
                        break;
                    }
                    bestVertex = (bestVertex + 1) % hole.nverts;
                }
            
                if(index == -1){
                    let region_outline = region.outline;
                    if(region_outline){
                        log_message("[Navmesh Baker] merge_holes: Failed to find merge points for " + region_outline.toString() + " and " + hole.toString() +".");
                    }
                    else{
                        log_message("[Navmesh Baker] merge_holes: Failed to find merge points for null and " + hole.toString() +".");
                    }
                    
                    continue;
                }
                if(!merge_contours(outline, hole, index, bestVertex)){
                    let region_outline = region.outline;
                    if(region_outline){
                        log_message("[Navmesh Baker] merge_holes: Failed to merge contours " + region_outline.toString() + " and " + hole.toString() + ".");
                    }
                    else{
                        log_message("[Navmesh Baker] merge_holes: Failed to merge contours null and " + hole.toString() + ".");
                    }
                    
                    continue;
                }
            }
        }
    }
}

export function build_contours(chf: CompactHeightfield,
                               max_error: float,
                               max_edge_len: int,
                               cset: ContourSet,
                               build_flags: int = RC_CONTOUR_TESS_WALL_EDGES): bool{
    const w: int = chf.width;
    const h: int = chf.height;
    const border_size: int = chf.border_size;

    let bmin: StaticArray<float> = new StaticArray<float>(3);
    bmin[0] = chf.bmin[0]; bmin[1] = chf.bmin[1]; bmin[2] = chf.bmin[2];
    let bmax: StaticArray<float> = new StaticArray<float>(3);
    bmax[0] = chf.bmax[0]; bmax[1] = chf.bmax[1]; bmax[2] = chf.bmax[2];
    if(border_size > 0){
        const pad: float = border_size * chf.cs;
        bmin[0] += pad;
        bmin[2] += pad;
        bmax[0] -= pad;
        bmax[2] -= pad;
    }
    cset.bmin = bmin;
    cset.bmax = bmax;
    cset.cs = chf.cs;
    cset.ch = chf.ch;
    cset.width = chf.width - chf.border_size * 2;
    cset.height = chf.height - chf.border_size * 2;
    cset.border_size = chf.border_size;
    cset.max_error = max_error;

    let max_contours: int = <i32>Math.max(chf.max_regions, 8);
    cset.conts = new StaticArray<Contour>(max_contours);
    for(let i = 0; i < max_contours; i++){
        cset.conts[i] = new Contour();
    }
    cset.nconts = 0;

    let flags: StaticArray<int> = new StaticArray<int>(chf.span_count);  // 1 byte per element

    let c: CompactCell | null = null;
    let s: CompactSpan | null = null;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    let res: int = 0;  // 1 byte
                    s = chf.spans[i];
                    if(s){
                        if((s.reg == 0) || (s.reg & RC_BORDER_REG)){
                            flags[i] = 0;
                            continue;
                        }
                        for(let dir = 0; dir < 4; dir++){
                            let r: int = 0;  // 2 bytes
                            if(get_con(s, dir) != RC_NOT_CONNECTED){
                                const ax: int = x + get_dir_offset_x(dir);
                                const ay: int = y + get_dir_offset_y(dir);
                                let cc: CompactCell | null = chf.cells[ax + ay*w];
                                if(cc){
                                    const ai: int = cc.index + get_con(s, dir);
                                    let ss: CompactSpan | null = chf.spans[ai];
                                    if(ss){
                                        r = ss.reg;
                                    }
                                }
                            }
                            if(r == s.reg){
                                res |= (1 << dir);
                            }
                        }
                    }
                    flags[i] = res ^ 0xf;  // Inverse, mark non connected edges
                }
            }
        }
    }

    let verts: List<int> = new List<int>();
    let simplified: List<int> = new List<int>();

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    if(flags[i] == 0 || flags[i] == 0xf){
                        flags[i] = 0;
                        continue;
                    }
                    s = chf.spans[i];
                    if(s){
                        const reg: int = s.reg;  // 2 bytes
                        if((reg == 0) || (reg & RC_BORDER_REG)){
                            continue;
                        }
                        const area: int = chf.areas[i];  // 1 byte

                        verts.reset();
                        simplified.reset();

                        walk_contour(x, y, i, chf, flags, verts);
                        simplify_contour(verts, simplified, max_error, max_edge_len, build_flags);
                        remove_degenerate_segments(simplified);

                        if(simplified.length / 4 >= 3){
                            if(cset.nconts > max_contours){
                                const old_max: int = max_contours;
                                max_contours *= 2;
                                let new_conts: StaticArray<Contour> = new StaticArray<Contour>(max_contours);
                                for(let j = 0; j < max_contours; j++){
                                    new_conts[j] = new Contour();
                                }
                                for(let j = 0; j < cset.nconts; j++){
                                    new_conts[j] = cset.conts[j];
                                    cset.conts[j].verts = new StaticArray<int>(0);
                                    cset.conts[j].rverts = new StaticArray<int>(0);
                                }
                                cset.conts = new_conts;
                            }

                            let cont: Contour = cset.conts[cset.nconts];
                            cset.nconts += 1;

                            cont.nverts = simplified.length / 4;
                            cont.verts = new StaticArray<int>(cont.nverts * 4);
                            for(let j = 0; j < cont.verts.length; j++){
                                cont.verts[j] = simplified[j];
                            }
                            if(border_size > 0){
                                for(let j = 0; j <cont.nverts; j++){
                                    cont.verts[4*j] -= border_size;
                                    cont.verts[4*j + 2] -= border_size;
                                }
                            }
                            cont.nrverts = verts.length / 4;
                            cont.rverts = new StaticArray<int>(cont.nrverts * 4);
                            for(let j = 0; j < cont.rverts.length; j++){
                                cont.rverts[j] = verts[j];
                            }
                            if(border_size > 0){
                                for(let j = 0; j < cont.nrverts; j++){
                                    cont.rverts[4*j] -= border_size;
                                    cont.rverts[4*j + 2] -= border_size;
                                }
                            }
                            cont.reg = reg;
                            cont.area = area;
                        }
                    }
                }
            }
        }
    }

    if(cset.nconts > 0){
        let winding: StaticArray<int> = new StaticArray<int>(cset.nconts);  // 2 bytes (signed char)
        let nholes: int = 0;
        for(let i = 0; i < cset.nconts; i++){
            let cont1: Contour = cset.conts[i];
            winding[i] = calc_area_of_polygon_2d(cont1.verts, cont1.nverts) < 0 ? -1 : 1;
            if(winding[i] < 0){
                nholes += 1;
            }
        }

        if(nholes > 0){
            const nregions: int = chf.max_regions + 1;
            let regions: StaticArray<ContourRegion> = new StaticArray<ContourRegion>(nregions);
            for(let i = 0; i < nregions; i++){
                regions[i] = new ContourRegion();
            }
            let holes: StaticArray<ContourHole> = new StaticArray<ContourHole>(cset.nconts);
            for(let i = 0; i < cset.nconts; i++){
                holes[i] = new ContourHole();
            }

            for(let i = 0; i < cset.nconts; i++){
                let cont2: Contour = cset.conts[i];
                if(winding[i] > 0){
                    if(regions[cont2.reg].outline){
                        log_message("[Navmesh Baker] build_contours: Multiple outlines for region " + cont2.reg.toString());
                    }
                    regions[cont2.reg].outline = cont2;
                }
                else{
                    regions[cont2.reg].nholes += 1;
                }
            }
            let index: int = 0;
            for(let i = 0; i < nregions; i++){
                if(regions[i].nholes > 0){
                    regions[i].holes_index = index;
                    index += regions[i].nholes;
                    regions[i].nholes = 0;
                }
            }
            for(let i = 0; i < cset.nconts; i++){
                let cont3: Contour = cset.conts[i];
                let co_reg: ContourRegion = regions[cont3.reg];
                if(winding[i] < 0){
                    holes[co_reg.holes_index + co_reg.nholes].contour = cont3;
                    co_reg.nholes += 1;
                }
            }

            for(let i = 0; i < nregions; i++){
                let c_reg: ContourRegion = regions[i];
                if(c_reg.nholes == 0){
                    continue;
                }
                if(c_reg.outline){
                    merge_region_holes(c_reg, holes);
                }
                else{
                    log_message("[Navmesh Baker] build_contours: Bad outline for region " + i.toString() + ", contour simplification is likely too aggressive");
                }
            }
        }
    }                    
    return true;
}
