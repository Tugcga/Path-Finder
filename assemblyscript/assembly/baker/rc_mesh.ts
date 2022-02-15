import { ContourSet, PolyMesh, Contour, Edge } from "./rc_classes";
import { RC_BORDER_VERTEX, RC_MESH_NULL_IDX, RC_MULTIPLE_REGS } from "./rc_constants";
import { next, prev, v_equal_static, build_int_pair, build_int_triple } from "./rc_calcs";
import { log_message } from "../common/utilities";

type int = i32;
type float = f64;
const VERTEX_BUCKET_COUNT: int = 1<<12;  // 4096

@inline
function area2(array: StaticArray<int>,
               a_ptr: int,
               b_ptr: int,
               c_ptr: int): int{
    return (array[b_ptr] - array[a_ptr]) * (array[c_ptr + 2] - array[a_ptr + 2]) - (array[c_ptr] - array[a_ptr]) * (array[b_ptr + 2] - array[a_ptr + 2]);
}

@inline
function collinear(array: StaticArray<int>,
                   a_ptr: int,
                   b_ptr: int,
                   c_ptr: int): bool{
    return area2(array, a_ptr, b_ptr, c_ptr) == 0;
}

@inline
function xorb(x: bool,
              y: bool): bool{
    return !((!x) == (!y));
}

@inline
function left(array: StaticArray<int>,
              a_ptr: int,
              b_ptr: int,
              c_ptr: int): bool{
    return area2(array, a_ptr, b_ptr, c_ptr) < 0;
}

@inline
function left_on(array: StaticArray<int>,
                 a_ptr: int,
                 b_ptr: int,
                 c_ptr: int): bool{
    return area2(array, a_ptr, b_ptr, c_ptr) <= 0;
}

@inline
function intersect_prop(array: StaticArray<int>,
                        a_ptr: int,
                        b_ptr: int,
                        c_ptr: int,
                        d_ptr: int): bool{
    if(collinear(array, a_ptr, b_ptr, c_ptr) ||
       collinear(array, a_ptr, b_ptr, d_ptr) ||
       collinear(array, c_ptr, d_ptr, a_ptr) ||
       collinear(array, c_ptr, d_ptr, b_ptr)){
        return false;
    }
    return xorb(left(array, a_ptr, b_ptr, c_ptr), left(array, a_ptr, b_ptr, d_ptr)) && xorb(left(array, c_ptr, d_ptr, a_ptr), left(array, c_ptr, d_ptr, b_ptr));
}

@inline
function between(array: StaticArray<int>,
                 a_ptr: int,
                 b_ptr: int,
                 c_ptr: int): bool{
    if(!collinear(array, a_ptr, b_ptr, c_ptr)){
        return false;
    }
    if(array[a_ptr] != array[b_ptr]){
        return ((array[a_ptr] <= array[c_ptr]) && (array[c_ptr] <= array[b_ptr])) || ((array[a_ptr] >= array[c_ptr]) && (array[c_ptr] >= array[b_ptr]));
    }
    else{
        return ((array[a_ptr + 2] <= array[c_ptr + 2]) && (array[c_ptr + 2] <= array[b_ptr + 2])) || ((array[a_ptr + 2] >= array[c_ptr + 2]) && (array[c_ptr + 2] >= array[b_ptr + 2]));
    }
}

@inline
function intersect(array: StaticArray<int>,
                   a_ptr: int,
                   b_ptr: int,
                   c_ptr: int,
                   d_ptr: int): bool{
    if(intersect_prop(array, a_ptr, b_ptr, c_ptr, d_ptr)){
        return true;
    }
    else if(between(array, a_ptr, b_ptr, c_ptr) || 
            between(array, a_ptr, b_ptr, d_ptr) || 
            between(array, c_ptr, d_ptr, a_ptr) || 
            between(array, c_ptr, d_ptr, b_ptr)){
        return true;
    }
    else{
        return false;
    }
}

function diagonalie(i: int,
                    j: int,
                    n: int,
                    verts: StaticArray<int>,
                    indices: StaticArray<int>): bool{
    const d0_ptr: int = (indices[i] & 0x0fffffff) * 4;
    const d1_ptr: int = (indices[j] & 0x0fffffff) * 4;
    
    for(let k = 0; k < n; k++){
        const k1: int = next(k, n);
        if(!((k == i) || (k1 == i) || (k == j) || (k1 == j))){
            const p0_ptr: int = (indices[k] & 0x0fffffff) * 4;
            const p1_ptr: int = (indices[k1] & 0x0fffffff) * 4;

            if(v_equal_static(d0_ptr, p0_ptr, verts) ||
               v_equal_static(d1_ptr, p0_ptr, verts) ||
               v_equal_static(d0_ptr, p1_ptr, verts) ||
               v_equal_static(d1_ptr, p1_ptr, verts)){
                continue;
            }
            
            if(intersect(verts, d0_ptr, d1_ptr, p0_ptr, p1_ptr)){
                return false;
            }
        }
    }
    return true;
}

@inline
function in_cone(i: int,
                 j: int,
                 n: int,
                 verts: StaticArray<int>,
                 indices: StaticArray<int>): bool{
    const pi: int = (indices[i] & 0x0fffffff) * 4;
    const pj: int = (indices[j] & 0x0fffffff) * 4;
    const pi1: int = (indices[next(i, n)] & 0x0fffffff) * 4;
    const pin1: int = (indices[prev(i, n)] & 0x0fffffff) * 4;

    if(left_on(verts, pin1, pi, pi1)){
        return left(verts, pi, pj, pin1) && left(verts, pj, pi, pi1);
    }
    return !(left_on(verts, pi, pj, pi1) && left_on(verts, pj, pi, pin1));
}

@inline
function diagonal(i: int,
                  j: int,
                  n: int,
                  verts: StaticArray<int>,
                  indices: StaticArray<int>): bool{
    return in_cone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

function diagonalie_loose(i: int,
                          j: int,
                          n: int,
                          verts: StaticArray<int>,
                          indices: StaticArray<int>): bool{
    const d0: int = (indices[i] & 0x0fffffff) * 4;
    const d1: int = (indices[j] & 0x0fffffff) * 4;
    
    for(let k = 0; k < n; k++){
        const k1: int = next(k, n);
        if(!((k == i) || (k1 == i) || (k == j) || (k1 == j))){
            const p0: int = (indices[k] & 0x0fffffff) * 4;
            const p1: int = (indices[k1] & 0x0fffffff) * 4;
            
            if(v_equal_static(d0, p0, verts) ||
               v_equal_static(d1, p0, verts) ||
               v_equal_static(d0, p1, verts) ||
               v_equal_static(d1, p1, verts)){
                continue;
            }
            
            if(intersect_prop(verts, d0, d1, p0, p1)){
                return false;
            }
        }
    }
    return true;
}

@inline
function in_cone_loose(i: int,
                       j: int,
                       n: int,
                       verts: StaticArray<int>,
                       indices: StaticArray<int>): bool{
    const pi: int = (indices[i] & 0x0fffffff) * 4;
    const pj: int = (indices[j] & 0x0fffffff) * 4;
    const pi1: int = (indices[next(i, n)] & 0x0fffffff) * 4;
    const pin1: int = (indices[prev(i, n)] & 0x0fffffff) * 4;
    
    if(left_on(verts, pin1, pi, pi1)){
        return left_on(verts, pi, pj, pin1) && left_on(verts, pj, pi, pi1);
    }
    return !(left_on(verts, pi, pj, pi1) && left_on(verts, pj, pi, pin1));
}

@inline
function diagonal_loose(i: int,
                        j: int,
                        n: int,
                        verts: StaticArray<int>,
                        indices: StaticArray<int>): bool{
    return in_cone_loose(i, j, n, verts, indices) && diagonalie_loose(i, j, n, verts, indices);
}

function triangulate(n: int,
                     verts: StaticArray<int>,
                     indices: StaticArray<int>,
                     tris: StaticArray<int>): int{
    let ntris: int = 0;
    let dst_ptr: int = 0;
    
    let i1: int = 0;
    let i2: int = 0;
    let i: int = 0;
    let p0_ptr: int = 0;
    let p2_ptr: int = 0;
    let dx: int = 0;
    let dy: int = 0;
    let length: int = 0;
    for(let i = 0; i < n; i++){
        i1 = next(i, n);
        i2 = next(i1, n);
        if(diagonal(i, i2, n, verts, indices)){
            indices[i1] |= 0x80000000;
        }
    }
    
    while(n > 3){
        let minLen: int = -1;
        let mini: int = -1;
        for(let i = 0; i < n; i++){
            i1 = next(i, n);
            if(indices[i1] & 0x80000000){
                p0_ptr = (indices[i] & 0x0fffffff) * 4;
                p2_ptr = (indices[next(i1, n)] & 0x0fffffff) * 4;
                
                dx = verts[p2_ptr] - verts[p0_ptr];
                dy = verts[p2_ptr + 2] - verts[p0_ptr + 2];
                length = dx*dx + dy*dy;
                
                if(minLen < 0 || length < minLen){
                    minLen = length;
                    mini = i;
                }
            }
        }
        
        if(mini == -1){
            minLen = -1;
            mini = -1;
            for(let i = 0; i < n; i++){
                i1 = next(i, n);
                i2 = next(i1, n);
                if(diagonal_loose(i, i2, n, verts, indices)){
                    p0_ptr = (indices[i] & 0x0fffffff) * 4;
                    p2_ptr = (indices[next(i2, n)] & 0x0fffffff) * 4;
                    dx = verts[p2_ptr] - verts[p0_ptr];
                    dy = verts[p2_ptr + 2] - verts[p0_ptr + 2];
                    length = dx*dx + dy*dy;
                    
                    if(minLen < 0 || length < minLen){
                        minLen = length;
                        mini = i;
                    }
                }
            }
            if(mini == -1){
                return -ntris;
            }
        }
        
        i = mini;
        i1 = next(i, n);
        i2 = next(i1, n);
        
        tris[dst_ptr] = indices[i] & 0x0fffffff;
        dst_ptr += 1;
        tris[dst_ptr] = indices[i1] & 0x0fffffff;
        dst_ptr += 1;
        tris[dst_ptr] = indices[i2] & 0x0fffffff;
        dst_ptr += 1;
        ntris += 1;
        
        n -= 1;
        for(let k = i1; k < n; k++){
            indices[k] = indices[k+1];
        }
        
        if(i1 >= n){
            i1 = 0;
        }
        i = prev(i1, n);
        if(diagonal(prev(i, n), i1, n, verts, indices)){
            indices[i] |= 0x80000000;
        }
        else{
            indices[i] &= 0x0fffffff;
        }
        
        if(diagonal(i, next(i1, n), n, verts, indices)){
            indices[i1] |= 0x80000000;
        }
        else{
            indices[i1] &= 0x0fffffff;
        }
    }
    
    tris[dst_ptr] = indices[0] & 0x0fffffff;
    dst_ptr += 1;
    tris[dst_ptr] = indices[1] & 0x0fffffff;
    dst_ptr += 1;
    tris[dst_ptr] = indices[2] & 0x0fffffff;
    dst_ptr += 1;
    ntris += 1;
    
    return ntris;
}

function compute_vertex_hash(x: int,
                             y: int,
                             z: int): int{
    const h1: int = 0x8da6b343;
    const h2: int = 0xd8163841;
    const h3: int = 0xcb1ab31f;
    const n: int = h1 * x + h2 * y + h3 * z;
    return n & (VERTEX_BUCKET_COUNT-1);
}

function add_vertex(x: int,  // all of then insigned 2 bytes
                    y: int,
                    z: int,
                    verts: StaticArray<int>,
                    first_vert: StaticArray<int>,
                    next_vert: StaticArray<int>,
                    nv: int): StaticArray<int>{  // unsigned 2 bytes, also return nv
    const bucket: int = compute_vertex_hash(x, 0, z);
    let i: int = first_vert[bucket];
    let v: int = 0;

    while(i != -1){
        v = i*3;
        if(verts[v] == x && (abs(verts[v + 1] - y) <= 2) && verts[v + 2] == z){
            return build_int_pair(i, nv);
        }
        i = next_vert[i];
    }
    
    i = nv;
    nv += 1;
    v = i*3;
    verts[v] = x;
    verts[v + 1] = y;
    verts[v + 2] = z;
    next_vert[i] = first_vert[bucket];
    first_vert[bucket] = i;
    
    return build_int_pair(i, nv);
}

function count_poly_verts(array: StaticArray<int>,
                          p: int,
                          nvp: int): int{
    for(let i = 0; i < nvp; i++){
        if(array[p + i] == RC_MESH_NULL_IDX){
            return i;
        }
    }
    return nvp;
}

function uleft(array: StaticArray<int>,
               a: int,  // all three unsigned 2 bytes
               b: int,
               c: int): bool{
    return (array[b] - array[a]) * (array[c + 2] - array[a + 2]) - (array[c] - array[a]) * (array[b + 2] - array[a + 2]) < 0;
}

function get_poly_merge_value(polys: StaticArray<int>,
                              pa: int,
                              pb: int,
                              verts: StaticArray<int>,
                              ea: int, 
                              eb: int,
                              nvp: int): StaticArray<int>{  // also return ea, eb
    const na: int = count_poly_verts(polys, pa, nvp);
    const nb: int = count_poly_verts(polys, pb, nvp);

    if(na+nb-2 > nvp)
        return build_int_triple(-1, ea, eb);
    
    ea = -1;
    eb = -1;
    
    for(let i = 0; i < na; i++){
        let va0: int = polys[pa + i];
        let va1: int = polys[pa + (i+1) % na];
        if(va0 > va1){
            const temp1 = va0;
            va0 = va1;
            va1 = temp1;
        }
        for(let j = 0; j < nb; j++){
            let vb0: int = polys[pb + j];
            let vb1: int = polys[pb + (j+1) % nb];
            if(vb0 > vb1){
                const temp2 = vb0;
                vb0 = vb1;
                vb1 = temp2;
            }
            if(va0 == vb0 && va1 == vb1){
                ea = i;
                eb = j;
                break;
            }
        }
    }

    if(ea == -1 || eb == -1){
        return build_int_triple(-1, ea, eb);
    }
    
    let va: int = 0;  // all three unsigned 2 bytes
    let vb: int = 0;
    let vc: int = 0;
    
    va = polys[pa + (ea+na-1) % na];
    vb = polys[pa + ea];
    vc = polys[pb + (eb+2) % nb];
    if(!uleft(verts, va*3, vb*3, vc*3)){
        return build_int_triple(-1, ea, eb);
    }
    
    va = polys[pb + (eb+nb-1) % nb];
    vb = polys[pb + eb];
    vc = polys[pa + (ea+2) % na];
    if(!uleft(verts, va*3, vb*3, vc*3)){
        return build_int_triple(-1, ea, eb);
    }
    
    va = polys[pa + ea];
    vb = polys[pa + (ea+1)%na];
    
    const dx: int = verts[va*3+0] - verts[vb*3+0];
    const dy: int = verts[va*3+2] - verts[vb*3+2];
    
    return build_int_triple(dx*dx + dy*dy, ea, eb);
}

function merge_poly_verts(polys: StaticArray<int>,
                          pa: int,
                          pb: int,
                          ea: int,
                          eb: int,
                          tmp: int,
                          nvp: int): void{
    const na: int = count_poly_verts(polys, pa, nvp);
    const nb: int = count_poly_verts(polys, pb, nvp);
    
    for(let i = 0; i < nvp; i++){
        polys[tmp + i] = 0xffff;
    }
    let n: int = 0;
    for(let i = 0; i < na - 1; i++){
        polys[tmp + n] = polys[pa + (ea+1+i) % na];
        n += 1;
    }
    for(let i = 0; i < nb - 1; i++){
        polys[tmp + n] = polys[pb + (eb+1+i) % nb];
        n += 1;
    }
    
    for(let i = 0; i < nvp; i++){
        polys[pa + i] = polys[tmp + i];
    }
}

function can_remove_vertex(mesh: PolyMesh,
                           rem: int): bool{
    const nvp: int = mesh.nvp;
    
    let numRemovedVerts: int = 0;
    let numTouchedVerts: int = 0;
    let numRemainingEdges: int = 0;
    let p: int = 0;
    let nv: int = 0;
    let j: int = 0;
    for(let i = 0; i < mesh.npolys; i++){
        p = i*nvp*2;
        nv = count_poly_verts(mesh.polys, p, nvp);
        let numRemoved: int = 0;
        let numVerts: int = 0;
        for(let j = 0; j < nv; j++){
            if(mesh.polys[p + j] == rem){
                numTouchedVerts += 1;
                numRemoved += 1;
            }
            numVerts += 1;
        }
        if(numRemoved > 0){
            numRemovedVerts += numRemoved;
            numRemainingEdges += numVerts - (numRemoved + 1);
        }
    }

    if(numRemainingEdges <= 2){
        return false;
    }
    
    const maxEdges: int = numTouchedVerts*2;
    let nedges: int = 0;
    let edges: StaticArray<int> = new StaticArray<int>(maxEdges*3);

    for(let i = 0; i < mesh.npolys; i++){
        p = i*nvp*2;
        nv = count_poly_verts(mesh.polys, p, nvp);

        let k: int = nv - 1;
        j = 0;
        while(j < nv){
            if(mesh.polys[p + j] == rem || mesh.polys[p + k] == rem){
                let a: int = mesh.polys[p + j];
                let b: int = mesh.polys[p + k];
                if(b == rem){
                    const temp1 = a;
                    a = b;
                    b = temp1;
                }
                    
                let e: int = 0;
                let exists: bool = false;
                for(let m = 0; m < nedges; m++){
                    e = m*3;
                    if(edges[e + 1] == b){
                        edges[e + 2] += 1;
                        exists = true;
                    }
                }
                if(!exists){
                    e = nedges*3;
                    edges[e] = a;
                    edges[e + 1] = b;
                    edges[e + 2] = 1;
                    nedges += 1;
                }
            }
            k = j;
            j += 1;
        }
    }
    let numOpenEdges: int = 0;
    for(let i = 0; i < nedges; i++){
        if(edges[i*3+2] < 2){
            numOpenEdges += 1;
        }
    }
    if(numOpenEdges > 2){
        return false;
    }
    
    return true;
}

function push_back(v: int,
                   array: StaticArray<int>,
                   an: int): int{
    array[an] = v;
    an += 1;
    return an;
}

function push_front(v: int,
                    arr: StaticArray<int>,
                    an: int): int{
    an += 1;
    for(let i = an - 1; i > 0; i--){
        arr[i] = arr[i - 1];
    }
    arr[0] = v;
    return an;
}

function remove_vertex(mesh: PolyMesh,
                       rem: int,
                       max_tris: int): bool{
    const nvp: int = mesh.nvp;

    let numRemovedVerts: int = 0;
    let i: int = 0;
    let p: int = 0;
    let nv: int = 0;
    let j: int = 0;
    for(let i = 0; i < mesh.npolys; i++){
        p = i*nvp*2;
        nv = count_poly_verts(mesh.polys, p, nvp);
        for(let j2 = 0; j2 < nv; j2++){
            if(mesh.polys[p + j2] == rem){
                numRemovedVerts += 1;
            }
        }
    }
    
    let nedges: int = 0;
    let edges: StaticArray<int> = new StaticArray<int>(numRemovedVerts*nvp*4);
    
    let nhole: int = 0;
    let hole: StaticArray<int> = new StaticArray<int>(numRemovedVerts*nvp);

    let nhreg: int = 0;
    let hreg: StaticArray<int> = new StaticArray<int>(numRemovedVerts*nvp);
    
    let nharea: int = 0;
    let harea: StaticArray<int> = new StaticArray<int>(numRemovedVerts*nvp);
    
    i = 0;
    while(i < mesh.npolys){
        p = i*nvp*2;
        nv = count_poly_verts(mesh.polys, p, nvp);
        let hasRem: int = false;
        for(let j2 = 0; j2 < nv; j2++){
            if(mesh.polys[p + j2] == rem){
                hasRem = true;
            }
        }
        if(hasRem){
            j = 0;
            let k: int = nv - 1;
            while(j < nv){
                if(mesh.polys[p + j] != rem && mesh.polys[p + k] != rem){
                    const e: int = nedges*4;
                    edges[e] = mesh.polys[p + k];
                    edges[e + 1] = mesh.polys[p + j];
                    edges[e + 2] = mesh.regs[i];
                    edges[e + 3] = mesh.areas[i];
                    nedges += 1;
                }
                k = j;
                j += 1;
            }
            const p2: int = (mesh.npolys-1)*nvp*2;
            if(p != p2){
                for(let m = 0; m < nvp; m++){
                    mesh.polys[p + m] = mesh.polys[p2 + m];
                }
            }
            for(let m = 0; m < nvp; m++){
                mesh.polys[p + nvp + m] = 0xffff;
            }
            mesh.regs[i] = mesh.regs[mesh.npolys-1];
            mesh.areas[i] = mesh.areas[mesh.npolys-1];
            mesh.npolys -= 1;
            i -= 1;
        }
        i += 1;
    }
    
    for(let i2 = rem; i2 < mesh.nverts - 1; i2++){
        mesh.verts[i2*3+0] = mesh.verts[(i2+1)*3+0];
        mesh.verts[i2*3+1] = mesh.verts[(i2+1)*3+1];
        mesh.verts[i2*3+2] = mesh.verts[(i2+1)*3+2];
    }
    mesh.nverts -= 1;

    for(let i2 = 0; i2 < mesh.npolys; i2++){
        p = i2*nvp*2;
        nv = count_poly_verts(mesh.polys, p, nvp);
        for(let j2 = 0; j2 < nv; j2++){
            if(mesh.polys[p + j2] > rem){
                mesh.polys[p + j2] -= 1;
            }
        }
    }
    for(let i2 = 0; i2 < nedges; i2++){
        if(edges[i2*4+0] > rem){
            edges[i2*4+0] -= 1;
        }
        if(edges[i2*4+1] > rem){
            edges[i2*4+1] -= 1;
        }
    }

    if(nedges == 0){
        return true;
    }

    nhole = push_back(edges[0], hole, nhole);
    nhreg = push_back(edges[2], hreg, nhreg);
    nharea = push_back(edges[3], harea, nharea);
    
    let ea: int = 0;
    let eb: int = 0;
    while(nedges > 0){
        let match: bool = false;
        
        i = 0;
        while(i < nedges){
            ea = edges[i*4+0];
            eb = edges[i*4+1];
            const r: int = edges[i*4+2];
            const a: int = edges[i*4+3];
            let add: bool = false;
            if(hole[0] == eb){
                nhole = push_front(ea, hole, nhole);
                nhreg = push_front(r, hreg, nhreg);
                nharea = push_front(a, harea, nharea);
                add = true;
            }
            else if(hole[nhole-1] == ea){
                nhole = push_back(eb, hole, nhole);
                nhreg = push_back(r, hreg, nhreg);
                nharea = push_back(a, harea, nharea);
                add = true;
            }
            if(add){
                edges[i*4+0] = edges[(nedges-1)*4+0];
                edges[i*4+1] = edges[(nedges-1)*4+1];
                edges[i*4+2] = edges[(nedges-1)*4+2];
                edges[i*4+3] = edges[(nedges-1)*4+3];
                nedges -= 1;
                match = true;
                i -= 1;
            }
            i += 1;
        }
        
        if(!match){
            break;
        }
    }

    let tris: StaticArray<int> = new StaticArray<int>(nhole*3);
    let tverts: StaticArray<int> = new StaticArray<int>(nhole*4);
    let thole: StaticArray<int> = new StaticArray<int>(nhole);
    
    for(let i2 = 0; i2 < nhole; i2++){
        const pi: int = hole[i2];
        tverts[i2*4+0] = mesh.verts[pi*3+0];
        tverts[i2*4+1] = mesh.verts[pi*3+1];
        tverts[i2*4+2] = mesh.verts[pi*3+2];
        tverts[i2*4+3] = 0;
        thole[i2] = i2;
    }

    let ntris: int = triangulate(nhole, tverts, thole, tris);
    if(ntris < 0){
        ntris = -ntris;
        log_message("[Navmesh Baker] remove_vertex: triangulate() returned bad results");
    }
   
    let polys: StaticArray<int> = new StaticArray<int>((ntris+1)*nvp);  // unsigned 2 bytes
    let pregs: StaticArray<int> = new StaticArray<int>(ntris);
    let pareas: StaticArray<int> = new StaticArray<int>(ntris)  // unsigned 1 byte
    const tmpPoly: int = ntris*nvp;
            
    let npolys: int = 0;
    for(let i2 = 0; i2 < ntris*nvp; i2++){
        polys[i2] = 0xffff;
    }
    for(let j2 = 0; j2 < ntris; j2++){
        const t: int = j2*3
        if(tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]){
            polys[npolys*nvp+0] = hole[tris[t]];
            polys[npolys*nvp+1] = hole[tris[t + 1]];
            polys[npolys*nvp+2] = hole[tris[t + 2]];

            if(hreg[tris[t]] != hreg[tris[t + 1]] || hreg[tris[t + 1]] != hreg[tris[t + 2]]){
                pregs[npolys] = RC_MULTIPLE_REGS;
            }
            else{
                pregs[npolys] = hreg[tris[t]];
            }

            pareas[npolys] = harea[tris[t]];
            npolys += 1;
        }
    }
    if(npolys == 0){
        return true;
    }
    
    if(nvp > 3){
        while(true){
            let bestMergeVal: int = 0;
            let bestPa: int = 0;
            let bestPb: int = 0;
            let bestEa: int = 0;
            let bestEb: int = 0;
            
            for(let j2 = 0; j2 < npolys - 1; j2++){
                const pj: int = j2*nvp;
                for(let k = j2 + 1; k < npolys; k++){
                    const pk: int = k*nvp;
                    ea = 0;
                    eb = 0;
                    let get_poly_merge_value_result = get_poly_merge_value(polys, pj, pk, mesh.verts, ea, eb, nvp);
                    const v = get_poly_merge_value_result[0];
                    ea = get_poly_merge_value_result[1];
                    eb = get_poly_merge_value_result[2];
                    if(v > bestMergeVal){
                        bestMergeVal = v;
                        bestPa = j2;
                        bestPb = k;
                        bestEa = ea;
                        bestEb = eb;
                    }
                }
            }
            
            if(bestMergeVal > 0){
                const pa: int = bestPa*nvp;
                const pb: int = bestPb*nvp;
                merge_poly_verts(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp);
                if(pregs[bestPa] != pregs[bestPb]){
                    pregs[bestPa] = RC_MULTIPLE_REGS;
                }

                const last: int = (npolys-1)*nvp;
                if(pb != last){
                    for(let m = 0; m < nvp; m++){
                        polys[pb + m] = polys[last + m];
                    }
                }
                pregs[bestPb] = pregs[npolys-1];
                pareas[bestPb] = pareas[npolys-1];
                npolys -= 1;
            }
            else{
                break;
            }
        }
    }
    
    for(let i2 = 0; i2 < npolys; i2++){
        if(mesh.npolys >= max_tris){
            break;
        }
        p = mesh.npolys*nvp*2;
        for(let m = 0; m < nvp * 2; m++){
            mesh.polys[p + m] = 0xffff;
        }
        for(let j2 = 0; j2 < nvp; j2++){
            mesh.polys[p + j2] = polys[i2*nvp+j2];
        }
        mesh.regs[mesh.npolys] = pregs[i2];
        mesh.areas[mesh.npolys] = pareas[i2];
        mesh.npolys += 1;
        if(mesh.npolys > max_tris){
            log_message("[Navmesh Baker]: remove_vertex: Too many polygons " + mesh.npolys.toString() + " (max: " + max_tris.toString() + ").")
            return false;
        }
    }

    return true;
}

function build_mesh_adjacency(polys: StaticArray<int>,
                              npolys: int,
                              nverts: int,
                              verts_per_poly: int): bool{
    const maxEdgeCount: int = npolys*verts_per_poly;
    let firstEdge: StaticArray<int> = new StaticArray<int>(nverts + maxEdgeCount);  // unsigned 2 bytes
    const nextEdge: int = nverts;
    let edgeCount: int = 0;
    
    let edges: StaticArray<Edge> = new StaticArray<Edge>(maxEdgeCount);
    for(let i = 0; i < maxEdgeCount; i++){
        edges[i] = new Edge();
    }
    
    for(let i = 0; i < nverts; i++){
        firstEdge[i] = RC_MESH_NULL_IDX;
    }
    
    let t: int = 0;
    let v0: int = 0;
    let v1: int = 0;
    for(let i = 0; i < npolys; i++){
        t = i*verts_per_poly*2;
        for(let j = 0; j < verts_per_poly; j++){
            if(polys[t + j] == RC_MESH_NULL_IDX){
                break;
            }
            v0 = polys[t + j];
            v1 = (j + 1 >= verts_per_poly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t] : polys[t + j + 1];
            if(v0 < v1){
                let edge: Edge = edges[edgeCount];
                edge.vert[0] = v0;
                edge.vert[1] = v1;
                edge.poly[0] = i;
                edge.poly_edge[0] = j;
                edge.poly[1] = i;
                edge.poly_edge[1] = 0;
                firstEdge[nextEdge + edgeCount] = firstEdge[v0];
                firstEdge[v0] = edgeCount;
                edgeCount += 1;
            }
        }
    }
    for(let i = 0; i < npolys; i++){
        t = i*verts_per_poly*2;
        for(let j = 0; j < verts_per_poly; j++){
            if(polys[t + j] == RC_MESH_NULL_IDX){
                break;
            }
            v0 = polys[t + j];
            v1 = (j + 1 >= verts_per_poly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t] : polys[t + j + 1];
            if(v0 > v1){
                let e: int = firstEdge[v1];
                while(e != RC_MESH_NULL_IDX){
                    let edge2: Edge = edges[e];
                    if(edge2.vert[1] == v0 && edge2.poly[0] == edge2.poly[1]){
                        edge2.poly[1] = i;
                        edge2.poly_edge[1] = j;
                        break;
                    }
                    e = firstEdge[nextEdge + e];
                }
            }
        }
    }

    for(let i = 0; i < edgeCount; i++){
        let edge_i: Edge = edges[i];
        if(edge_i.poly[0] != edge_i.poly[1]){
            const p0: int = edge_i.poly[0]*verts_per_poly*2;
            const p1: int = edge_i.poly[1]*verts_per_poly*2;
            polys[p0 + verts_per_poly + edge_i.poly_edge[0]] = edge_i.poly[1];
            polys[p1 + verts_per_poly + edge_i.poly_edge[1]] = edge_i.poly[0];
        }
    }
    
    return true;
}

export function build_poly_mesh(cset: ContourSet,
                                nvp: int,
                                mesh: PolyMesh): bool{
    mesh.bmin = cset.bmin;
    mesh.bmax = cset.bmax;
    mesh.cs = cset.cs;
    mesh.ch = cset.ch;
    mesh.border_size = cset.border_size;
    mesh.max_edge_error = cset.max_error;

    let max_vertices: int = 0;
    let max_tris: int = 0;
    let max_verts_per_cont: int = 0;
    for(let i = 0; i < cset.nconts; i++){
        if(cset.conts[i].nverts < 3){
            continue;
        }
        max_vertices += cset.conts[i].nverts;
        max_tris += cset.conts[i].nverts - 2;
        max_verts_per_cont = <i32>Math.max(max_verts_per_cont, cset.conts[i].nverts);
    }
    
    if(max_vertices >= 0xfffe){  // 65534
        log_message("[Navmesh Baker] build_poly_mesh: Too many vertices " + max_vertices.toString());
        return false;
    }

    let vflags: StaticArray<int> = new StaticArray<int>(max_vertices);  // 1 byte per element
    mesh.verts = new StaticArray<int>(max_vertices * 3);  // 2 bytes
    mesh.polys = new StaticArray<int>(max_tris * nvp * 2);
    for(let i = 0; i < max_tris * nvp * 2; i++){
        mesh.polys[i] = 65535;  // 2 bytes
    }
    mesh.regs = new StaticArray<int>(max_tris);  // 2 bytes
    mesh.areas = new StaticArray<int>(max_tris);  // 2 bytes

    mesh.nverts = 0;
    mesh.npolys = 0;
    mesh.nvp = nvp;
    mesh.maxpolys = max_tris;

    let next_vert: StaticArray<int> = new StaticArray<int>(max_vertices);
    let first_vert: StaticArray<int> = new StaticArray<int>(VERTEX_BUCKET_COUNT);
    for(let i = 0; i < VERTEX_BUCKET_COUNT; i++){
        first_vert[i] = -1;
    }
    let indices: StaticArray<int> = new StaticArray<int>(max_verts_per_cont);
    let tris: StaticArray<int> = new StaticArray<int>(max_verts_per_cont * 3);
    let polys: StaticArray<int> = new StaticArray<int>((max_verts_per_cont + 1) * nvp);  // 2 unsigned bytes
    const tmp_poly: int = max_verts_per_cont * nvp;

    let k: int = 0;
    for(let i = 0; i < cset.nconts; i++){
        let cont: Contour = cset.conts[i];
        
        if(cont.nverts < 3){
            continue;
        }
        
        for(let j = 0; j < cont.nverts; j++){
            indices[j] = j;
        }
            
        let ntris: int = triangulate(cont.nverts, cont.verts, indices, tris);

        if(ntris <= 0){
            ntris = -ntris;
        }
                
        for(let j = 0; j < cont.nverts; j++){
            let add_vertex_result = add_vertex(cont.verts[j*4], cont.verts[j*4 + 1], cont.verts[j*4 + 2], mesh.verts, first_vert, next_vert, mesh.nverts);
            indices[j] = add_vertex_result[0];
            mesh.nverts = add_vertex_result[1];
            if(cont.verts[j*4 + 3] & RC_BORDER_VERTEX){
                vflags[indices[j]] = 1;
            }
        }

        let npolys: int = 0;
        for(let k2 = 0; k2 < max_verts_per_cont * nvp; k2++){
            polys[k2] = 0xffff;  // 65535
        }

        for(let j = 0; j < ntris; j++){
            if(tris[j*3] != tris[j*3 + 1] && tris[j*3] != tris[j*3 + 2] && tris[j*3 + 1] != tris[j*3 + 2]){
                polys[npolys*nvp+0] = indices[tris[j*3]];
                polys[npolys*nvp+1] = indices[tris[j*3 + 1]];
                polys[npolys*nvp+2] = indices[tris[j*3 + 2]];
                npolys += 1;
            }
        }
        if(npolys == 0){
            continue;
        }

        if(nvp > 3){
            while(true){
                let bestMergeVal: int = 0;
                let bestPa: int = 0;
                let bestPb: int = 0;
                let bestEa: int = 0;
                let bestEb: int = 0;
                
                let jw = 0;
                while(jw < npolys - 1){
                    const pj_ptr: int = jw*nvp;
                    k = jw + 1;
                    while(k < npolys){
                        const pk_ptr: int = k*nvp;
                        let ea: int = 0;
                        let eb: int = 0;
                        let get_poly_merge_value_result = get_poly_merge_value(polys, pj_ptr, pk_ptr, mesh.verts, ea, eb, nvp);
                        const v = get_poly_merge_value_result[0];
                        ea = get_poly_merge_value_result[1];
                        eb = get_poly_merge_value_result[2];
                        if(v > bestMergeVal){
                            bestMergeVal = v;
                            bestPa = jw;
                            bestPb = k;
                            bestEa = ea;
                            bestEb = eb;
                        }
                        k += 1;
                    }
                    jw += 1;
                }
                
                if(bestMergeVal > 0){
                    const pa_ptr: int = bestPa*nvp;
                    const pb_ptr: int = bestPb*nvp;
                    merge_poly_verts(polys, pa_ptr, pb_ptr, bestEa, bestEb, tmp_poly, nvp);
                    const lastPoly_ptr: int = (npolys-1)*nvp;
                    if(pb_ptr != lastPoly_ptr){
                        for(let s = 0; s < nvp; s++){
                            polys[pb_ptr + s] = polys[lastPoly_ptr + s];
                        }
                    }
                    npolys -= 1;
                }
                else{
                    break;
                }
            }
        }
        
        for(let j = 0; j < npolys; j++){
            const p_ptr: int = mesh.npolys*nvp*2;
            const q_ptr: int = j*nvp;
            for(let k2 = 0; k2 < nvp; k2++){
                mesh.polys[p_ptr + k2] = polys[q_ptr + k2];
            }
            mesh.regs[mesh.npolys] = cont.reg;
            mesh.areas[mesh.npolys] = cont.area;
            mesh.npolys += 1;
            if(mesh.npolys > max_tris){
                return false;
            }
        }
    }

    let iw = 0;
    while(iw < mesh.nverts){
        if(vflags[iw] > 0){
            if(!can_remove_vertex(mesh, iw)){
                iw += 1;
                continue;
            }
            if(!remove_vertex(mesh, iw, max_tris)){
                log_message("[Navmesh Baker] build_poly_mesh: Failed to remove edge vertex " + iw.toString());
                return false;
            }
            for(let j = iw; j < mesh.nverts; j++){
                vflags[j] = vflags[j+1];
            }
            iw -= 1;
        }
        iw += 1;
    }
    
    if(!build_mesh_adjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp)){
        log_message("[Navmesh Baker] build_poly_mesh: Adjacency failed");
        return false;
    }
    
    if(mesh.border_size > 0){
        const w: int = cset.width;
        const h: int = cset.height;
        for(let i = 0; i < mesh.npolys; i++){
            const p: int = i*2*nvp;
            for(let j = 0; j < nvp; j++){
                if(mesh.polys[p + j] == RC_MESH_NULL_IDX){
                    break;
                }
                if(mesh.polys[p + nvp+j] != RC_MESH_NULL_IDX){
                    continue;
                }
                let nj: int = j+1;
                if(nj >= nvp || mesh.polys[p + nj] == RC_MESH_NULL_IDX){
                    nj = 0;
                }
                const va: int = mesh.polys[p+j]*3;
                const vb: int = mesh.polys[p+nj]*3;

                if(mesh.verts[va] == 0 && mesh.verts[vb] == 0){
                    mesh.polys[p + nvp+j] = 0x8000 | 0;
                }
                else if(mesh.verts[va + 2] == h && mesh.verts[vb + 2] == h){
                    mesh.polys[p + nvp+j] = 0x8000 | 1;
                }
                else if(mesh.verts[va] == w && mesh.verts[vb] == w){
                    mesh.polys[p + nvp+j] = 0x8000 | 2;
                }
                else if(mesh.verts[va + 2] == 0 && mesh.verts[vb + 2] == 0){
                    mesh.polys[p + nvp+j] = 0x8000 | 3;
                }
            }
        }
    }

    if(mesh.nverts > 0xffff){
        log_message("[Navmesh Baker] build_poly_mesh: The resulting mesh has too many vertices " + mesh.nverts.toString() + " (max " + 0xffff.toString() + "). Data can be corrupted");
    }
    if(mesh.npolys > 0xffff){
        log_message("[Navmesh Baker] build_poly_mesh: The resulting mesh has too many polygons " + mesh.npolys.toString() + " (max " + 0xffff.toString() + "). Data can be corrupted");
    }

    return true;
}
