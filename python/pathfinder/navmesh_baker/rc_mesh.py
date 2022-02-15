from typing import List, Tuple
from pathfinder.navmesh_baker.rc_classes import ContourSet, PolyMesh, Contour, Edge
from pathfinder.navmesh_baker.rc_constants import RC_BORDER_VERTEX, RC_MESH_NULL_IDX, RC_MULTIPLE_REGS
from pathfinder.navmesh_baker.rc_calcs import next, prev, v_equal

VERTEX_BUCKET_COUNT = 1<<12  # 4096

def area2(array: List[int],
          a_ptr: int,
          b_ptr: int,
          c_ptr: int) -> int:
    return (array[b_ptr] - array[a_ptr]) * (array[c_ptr + 2] - array[a_ptr + 2]) - (array[c_ptr] - array[a_ptr]) * (array[b_ptr + 2] - array[a_ptr + 2])

def collinear(array: List[int],
              a_ptr: int,
              b_ptr: int,
              c_ptr: int) -> bool:
    return area2(array, a_ptr, b_ptr, c_ptr) == 0

def xorb(x: bool,
         y: bool) -> bool:
    return (not x) ^ (not y)

def left(array: List[int],
         a_ptr: int,
         b_ptr: int,
         c_ptr: int) -> bool:
    return area2(array, a_ptr, b_ptr, c_ptr) < 0

def left_on(array: List[int],
            a_ptr: int,
            b_ptr: int,
            c_ptr: int) -> bool:
    return area2(array, a_ptr, b_ptr, c_ptr) <= 0

def intersect_prop(array: List[int],
                   a_ptr: int,
                   b_ptr: int,
                   c_ptr: int,
                   d_ptr: int) -> bool:
    if collinear(array, a_ptr, b_ptr, c_ptr) or collinear(array, a_ptr, b_ptr, d_ptr) or collinear(array, c_ptr, d_ptr, a_ptr) or collinear(array, c_ptr, d_ptr, b_ptr):
        return False
    return xorb(left(array, a_ptr, b_ptr, c_ptr), left(array, a_ptr, b_ptr, d_ptr)) and xorb(left(array, c_ptr, d_ptr, a_ptr), left(array, c_ptr, d_ptr, b_ptr))

def between(array: List[int],
            a_ptr: int,
            b_ptr: int,
            c_ptr: int) -> bool:
    if not collinear(array, a_ptr, b_ptr, c_ptr):
        return False
    if array[a_ptr] != array[b_ptr]:
        return ((array[a_ptr] <= array[c_ptr]) and (array[c_ptr] <= array[b_ptr])) or ((array[a_ptr] >= array[c_ptr]) and (array[c_ptr] >= array[b_ptr]))
    else:
        return ((array[a_ptr + 2] <= array[c_ptr + 2]) and (array[c_ptr + 2] <= array[b_ptr + 2])) or ((array[a_ptr + 2] >= array[c_ptr + 2]) and (array[c_ptr + 2] >= array[b_ptr + 2]))

def intersect(array: List[int],
              a_ptr: int,
              b_ptr: int,
              c_ptr: int,
              d_ptr: int) -> bool:
    if intersect_prop(array, a_ptr, b_ptr, c_ptr, d_ptr):
        return True
    elif between(array, a_ptr, b_ptr, c_ptr) or between(array, a_ptr, b_ptr, d_ptr) or between(array, c_ptr, d_ptr, a_ptr) or between(array, c_ptr, d_ptr, b_ptr):
        return True
    else:
        return False

def diagonalie(i: int,
               j: int,
               n: int,
               verts: List[int],
               indices: List[int]) -> bool:
    d0_ptr: int = (indices[i] & 0x0fffffff) * 4  # pointer to verts
    d1_ptr: int = (indices[j] & 0x0fffffff) * 4
    
    # For each edge (k,k+1) of P
    for k in range(n):
        k1: int = next(k, n)
        # Skip edges incident to i or j
        if not((k == i) or (k1 == i) or (k == j) or (k1 == j)):
            p0_ptr: int = (indices[k] & 0x0fffffff) * 4
            p1_ptr: int = (indices[k1] & 0x0fffffff) * 4

            if v_equal(d0_ptr, p0_ptr, verts) or v_equal(d1_ptr, p0_ptr, verts) or v_equal(d0_ptr, p1_ptr, verts) or v_equal(d1_ptr, p1_ptr, verts):
                continue
            
            if intersect(verts, d0_ptr, d1_ptr, p0_ptr, p1_ptr):
                return False
    return True

def in_cone(i: int,
            j: int,
            n: int,
            verts: List[int],
            indices: List[int]) -> bool:
    pi: int = (indices[i] & 0x0fffffff) * 4
    pj: int = (indices[j] & 0x0fffffff) * 4
    pi1: int = (indices[next(i, n)] & 0x0fffffff) * 4
    pin1: int = (indices[prev(i, n)] & 0x0fffffff) * 4

    # If P[i] is a convex vertex [ i+1 left or on (i-1,i) ]
    if left_on(verts, pin1, pi, pi1):
        return left(verts, pi, pj, pin1) and left(verts, pj, pi, pi1)
    # Assume (i-1,i,i+1) not collinear
    # else P[i] is reflex
    return not(left_on(verts, pi, pj, pi1) and left_on(verts, pj, pi, pin1))

def diagonal(i: int,
             j: int,
             n: int,
             verts: List[int],
             indices: List[int]) -> bool:
    return in_cone(i, j, n, verts, indices) and diagonalie(i, j, n, verts, indices)

def diagonalie_loose(i: int,
                     j: int,
                     n: int,
                     verts: List[int],
                     indices: List[int]) -> bool:
    d0: int = (indices[i] & 0x0fffffff) * 4
    d1: int = (indices[j] & 0x0fffffff) * 4
    
    # For each edge (k,k+1) of P
    for k in range(n):
        k1: int = next(k, n)
        # Skip edges incident to i or j
        if not((k == i) or (k1 == i) or (k == j) or (k1 == j)):
            p0: int = (indices[k] & 0x0fffffff) * 4
            p1: int = (indices[k1] & 0x0fffffff) * 4
            
            if v_equal(d0, p0, verts) or v_equal(d1, p0, verts) or v_equal(d0, p1, verts) or v_equal(d1, p1, verts):
                continue
            
            if intersect_prop(verts, d0, d1, p0, p1):
                return False
    return True

def in_cone_loose(i: int,
                  j: int,
                  n: int,
                  verts: List[int],
                  indices: List[int]) -> bool:
    pi: int = (indices[i] & 0x0fffffff) * 4
    pj: int = (indices[j] & 0x0fffffff) * 4
    pi1: int = (indices[next(i, n)] & 0x0fffffff) * 4
    pin1: int = (indices[prev(i, n)] & 0x0fffffff) * 4
    
    # If P[i] is a convex vertex [ i+1 left or on (i-1,i) ]
    if left_on(verts, pin1, pi, pi1):
        return left_on(verts, pi, pj, pin1) and left_on(verts, pj, pi, pi1)
    # Assume (i-1,i,i+1) not collinear
    # else P[i] is reflex
    return not(left_on(verts, pi, pj, pi1) and left_on(verts, pj, pi, pin1))

def diagonal_loose(i: int,
                   j: int,
                   n: int,
                   verts: List[int],
                   indices: List[int]) -> bool:
    return in_cone_loose(i, j, n, verts, indices) and diagonalie_loose(i, j, n, verts, indices)

def triangulate(n: int,
                verts: List[int],
                indices: List[int],
                tris: List[int]) -> int:
    ntris: int = 0
    dst_ptr: int = 0  # pointer to tris array
    
    i1: int = 0
    i2: int = 0
    i: int = 0
    p0_ptr: int = 0
    p2_ptr: int = 0
    dx: int = 0
    dy: int = 0
    length: int = 0
    # The last bit of the index is used to indicate if the vertex can be removed
    for i in range(n):
        i1 = next(i, n)
        i2 = next(i1, n)
        if diagonal(i, i2, n, verts, indices):
            indices[i1] |= 0x80000000
    
    while n > 3:
        minLen: int = -1
        mini: int = -1
        for i in range(n):
            i1 = next(i, n)
            if indices[i1] & 0x80000000:
                p0_ptr = (indices[i] & 0x0fffffff) * 4
                p2_ptr = (indices[next(i1, n)] & 0x0fffffff) * 4
                
                dx = verts[p2_ptr] - verts[p0_ptr]
                dy = verts[p2_ptr + 2] - verts[p0_ptr + 2]
                length = dx*dx + dy*dy
                
                if minLen < 0 or length < minLen:
                    minLen = length
                    mini = i
        
        if mini == -1:
            minLen = -1
            mini = -1
            for i in range(n):
                i1 = next(i, n)
                i2 = next(i1, n)
                if diagonal_loose(i, i2, n, verts, indices):
                    p0_ptr = (indices[i] & 0x0fffffff) * 4
                    p2_ptr = (indices[next(i2, n)] & 0x0fffffff) * 4
                    dx = verts[p2_ptr] - verts[p0_ptr]
                    dy = verts[p2_ptr + 2] - verts[p0_ptr + 2]
                    length = dx*dx + dy*dy
                    
                    if minLen < 0 or length < minLen:
                        minLen = length
                        mini = i
            if mini == -1:
                # The contour is messed up. This sometimes happens
                # if the contour simplification is too aggressive
                return -ntris
        
        i = mini
        i1 = next(i, n)
        i2 = next(i1, n)
        
        tris[dst_ptr] = indices[i] & 0x0fffffff
        dst_ptr += 1
        tris[dst_ptr] = indices[i1] & 0x0fffffff
        dst_ptr += 1
        tris[dst_ptr] = indices[i2] & 0x0fffffff
        dst_ptr += 1
        ntris += 1
        
        # Removes P[i1] by copying P[i+1]...P[n-1] left one index
        n -= 1
        for k in range(i1, n):
            indices[k] = indices[k+1]
        
        if i1 >= n:
            i1 = 0
        i = prev(i1, n)
        # Update diagonal flags
        if diagonal(prev(i, n), i1, n, verts, indices):
            indices[i] |= 0x80000000
        else:
            indices[i] &= 0x0fffffff
        
        if diagonal(i, next(i1, n), n, verts, indices):
            indices[i1] |= 0x80000000
        else:
            indices[i1] &= 0x0fffffff
    
    # Append the remaining triangle

    tris[dst_ptr] = indices[0] & 0x0fffffff
    dst_ptr += 1
    tris[dst_ptr] = indices[1] & 0x0fffffff
    dst_ptr += 1
    tris[dst_ptr] = indices[2] & 0x0fffffff
    dst_ptr += 1
    ntris += 1
    
    return ntris;

def compute_vertex_hash(x: int,
                        y: int,
                        z: int) -> int:
    # all values here unsigned
    h1: int = 0x8da6b343  # Large multiplicative constants;
    h2: int = 0xd8163841  # here arbitrarily chosen primes
    h3: int = 0xcb1ab31f
    n: int = h1 * x + h2 * y + h3 * z
    return n & (VERTEX_BUCKET_COUNT-1)

def add_vertex(x: int,  # all of then insigned 2 bytes
               y: int,
               z: int,
               verts: List[int],
               first_vert: List[int],
               next_vert: List[int],
               nv: int) -> Tuple[int, int]:  # unsigned 2 bytes, also return nv
    bucket: int = compute_vertex_hash(x, 0, z)
    i: int = first_vert[bucket]
    v: int = 0

    while i != -1:
        v = i*3  # pointer to verts
        if verts[v] == x and (abs(verts[v + 1] - y) <= 2) and verts[v + 2] == z:
            return (i, nv)
        i = next_vert[i] # next
    
    # Could not find, create new
    i = nv
    nv += 1
    v = i*3
    verts[v] = x
    verts[v + 1] = y
    verts[v + 2] = z
    next_vert[i] = first_vert[bucket]
    first_vert[bucket] = i
    
    return (i, nv)

def count_poly_verts(array: List[int],
                     p: int,
                     nvp: int) -> int:
    for i in range(nvp):
        if (array[p + i] == RC_MESH_NULL_IDX):
            return i
    return nvp

def uleft(array: List[int],
          a: int,  # all three unsigned 2 bytes
          b: int,
          c: int) -> bool:
    return (array[b] - array[a]) * (array[c + 2] - array[a + 2]) - (array[c] - array[a]) * (array[b + 2] - array[a + 2]) < 0

def get_poly_merge_value(polys: List[int],
                         pa: int,
                         pb: int,
                         verts: List[int],
                         ea: int, 
                         eb: int,
                         nvp: int) -> Tuple[int, int, int]:  # also return ea, eb
    na: int = count_poly_verts(polys, pa, nvp)
    nb: int = count_poly_verts(polys, pb, nvp)

    # If the merged polygon would be too big, do not merge
    if na+nb-2 > nvp:
        return (-1, ea, eb)
    
    # Check if the polygons share an edge
    ea = -1
    eb = -1
    
    for i in range(na):
        va0: int = polys[pa + i]
        va1: int = polys[pa + (i+1) % na]
        if va0 > va1:
            va0, va1 = va1, va0
        for j in range(nb):
            vb0: int = polys[pb + j]
            vb1: int = polys[pb + (j+1) % nb]
            if vb0 > vb1:
                vb0, vb1 = vb1, vb0
            if va0 == vb0 and va1 == vb1:
                ea = i
                eb = j
                break

    # No common edge, cannot merge
    if ea == -1 or eb == -1:
        return (-1, ea, eb)
    
    # Check to see if the merged polygon would be convex
    va: int = 0  # all three unsigned 2 bytes
    vb: int = 0
    vc: int = 0
    
    va = polys[pa + (ea+na-1) % na]
    vb = polys[pa + ea]
    vc = polys[pb + (eb+2) % nb]
    if not uleft(verts, va*3, vb*3, vc*3):
        return (-1, ea, eb)
    
    va = polys[pb + (eb+nb-1) % nb]
    vb = polys[pb + eb]
    vc = polys[pa + (ea+2) % na]
    if not uleft(verts, va*3, vb*3, vc*3):
        return (-1, ea, eb)
    
    va = polys[pa + ea]
    vb = polys[pa + (ea+1)%na]
    
    dx: int = verts[va*3+0] - verts[vb*3+0]
    dy: int = verts[va*3+2] - verts[vb*3+2]
    
    return (dx*dx + dy*dy, ea, eb)

def merge_poly_verts(polys: List[int],
                     pa: int,
                     pb: int,
                     ea: int,
                     eb: int,
                     tmp: int,  # temp pointer to the poly array
                     nvp: int):
    na: int = count_poly_verts(polys, pa, nvp)
    nb: int = count_poly_verts(polys, pb, nvp)
    
    # Merge polygons
    for i in range(nvp):
        polys[tmp + i] = 0xffff
    n: int = 0
    # Add pa
    for i in range(na - 1):
        polys[tmp + n] = polys[pa + (ea+1+i) % na]
        n += 1
    # Add pb
    for i in range(nb - 1):
        polys[tmp + n] = polys[pb + (eb+1+i) % nb]
        n += 1
    
    for i in range(nvp):
        polys[pa + i] = polys[tmp + i]

def can_remove_vertex(mesh: PolyMesh,
                      rem: int) -> bool:
    nvp: int = mesh.nvp
    
    # Count number of polygons to remove
    numRemovedVerts: int = 0
    numTouchedVerts: int = 0
    numRemainingEdges: int = 0
    p: int = 0
    nv: int = 0
    j: int = 0
    for i in range(mesh.npolys):
        p = i*nvp*2  # pointer to mesh.polys
        nv = count_poly_verts(mesh.polys, p, nvp)
        numRemoved: int = 0
        numVerts: int = 0
        for j in range(nv):
            if mesh.polys[p + j] == rem:
                numTouchedVerts += 1
                numRemoved += 1
            numVerts += 1
        if numRemoved > 0:
            numRemovedVerts += numRemoved
            numRemainingEdges += numVerts - (numRemoved + 1)
    
    # There would be too few edges remaining to create a polygon
    # This can happen for example when a tip of a triangle is marked
    # as deletion, but there are no other polys that share the vertex
    # In this case, the vertex should not be removed
    if numRemainingEdges <= 2:
        return False
    
    # Find edges which share the removed vertex
    maxEdges: int = numTouchedVerts*2
    nedges: int = 0
    edges: List[int] = [0] * (maxEdges*3)

    for i in range(mesh.npolys):
        p = i*nvp*2
        nv = count_poly_verts(mesh.polys, p, nvp)

        # Collect edges which touches the removed vertex
        k: int = nv - 1
        j = 0
        while j < nv:
            if mesh.polys[p + j] == rem or mesh.polys[p + k] == rem:
                # Arrange edge so that a=rem
                a: int = mesh.polys[p + j]
                b: int = mesh.polys[p + k]
                if b == rem:
                    a, b = b, a
                    
                # Check if the edge exists
                e: int = 0
                exists: bool = False
                for m in range(nedges):
                    e = m*3  # pointer to edges
                    if edges[e + 1] == b:
                        # Exists, increment vertex share count
                        edges[e + 2] += 1
                        exists = True
                # Add new edge
                if not exists:
                    e = nedges*3
                    edges[e] = a
                    edges[e + 1] = b
                    edges[e + 2] = 1
                    nedges += 1
            k = j
            j += 1

    # There should be no more than 2 open edges
    # This catches the case that two non-adjacent polygons
    # share the removed vertex. In that case, do not remove the vertex
    numOpenEdges: int = 0
    for i in range(nedges):
        if edges[i*3+2] < 2:
            numOpenEdges += 1
    if numOpenEdges > 2:
        return False
    
    return True

def push_back(v: int,
              array: List[int],
              an: int) -> int:  # return an
    array[an] = v
    an += 1
    return an

def push_front(v: int,
               arr: List[int],
               an: int) -> int:  # return an
    an += 1
    for i in range(an - 1, 0, -1):
        arr[i] = arr[i - 1]
    arr[0] = v
    return an

def remove_vertex(mesh: PolyMesh,
                  rem: int,
                  max_tris: int) -> bool:
    nvp: int = mesh.nvp

    # Count number of polygons to remove
    numRemovedVerts: int = 0
    i: int = 0
    p: int = 0
    nv: int = 0
    j: int = 0
    for i in range(mesh.npolys):
        p = i*nvp*2  # poijter to mesh.polys
        nv = count_poly_verts(mesh.polys, p, nvp)
        for j in range(nv):
            if (mesh.polys[p + j] == rem):
                numRemovedVerts += 1
    
    nedges: int = 0
    edges: List[int] = [0] * (numRemovedVerts*nvp*4)
    
    nhole: int = 0
    hole: List[int] = [0] * (numRemovedVerts*nvp)

    nhreg: int = 0
    hreg: List[int] = [0] * (numRemovedVerts*nvp)
    
    nharea: int = 0
    harea: List[int] = [0] * (numRemovedVerts*nvp)
    
    i = 0
    while i < mesh.npolys:
        p = i*nvp*2  # pointer to mesh.polys
        nv = count_poly_verts(mesh.polys, p, nvp)
        hasRem: int = False
        for j in range(nv):
            if mesh.polys[p + j] == rem:
                hasRem = True
        if hasRem:
            # Collect edges which does not touch the removed vertex
            j = 0
            k: int = nv - 1
            while j < nv:
                if mesh.polys[p + j] != rem and mesh.polys[p + k] != rem:
                    e: int = nedges*4  # pointer to edges
                    edges[e] = mesh.polys[p + k]
                    edges[e + 1] = mesh.polys[p + j]
                    edges[e + 2] = mesh.regs[i]
                    edges[e + 3] = mesh.areas[i]
                    nedges += 1
                k = j
                j += 1
            # Remove the polygon
            p2: int = (mesh.npolys-1)*nvp*2
            if p != p2:
                for m in range(nvp):
                    mesh.polys[p + m] = mesh.polys[p2 + m]
            for m in range(nvp):
                mesh.polys[p + nvp + m] = 0xffff
            mesh.regs[i] = mesh.regs[mesh.npolys-1]
            mesh.areas[i] = mesh.areas[mesh.npolys-1]
            mesh.npolys -= 1
            i -= 1
        i += 1
    
    # Remove vertex
    for i in range(rem, mesh.nverts - 1):
        mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0]
        mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1]
        mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2]
    mesh.nverts -= 1

    # Adjust indices to match the removed vertex layout
    for i in range(mesh.npolys):
        p = i*nvp*2
        nv = count_poly_verts(mesh.polys, p, nvp)
        for j in range(nv):
            if mesh.polys[p + j] > rem:
                mesh.polys[p + j] -= 1
    for i in range(nedges):
        if edges[i*4+0] > rem:
            edges[i*4+0] -= 1
        if edges[i*4+1] > rem:
            edges[i*4+1] -= 1

    if nedges == 0:
        return True

    # Start with one vertex, keep appending connected
    # segments to the start and end of the hole
    nhole = push_back(edges[0], hole, nhole)
    nhreg = push_back(edges[2], hreg, nhreg)
    nharea = push_back(edges[3], harea, nharea)
    
    ea: int = 0
    eb: int = 0
    while nedges > 0:
        match: bool = False
        
        i = 0
        while i < nedges:
            ea = edges[i*4+0]
            eb = edges[i*4+1]
            r: int = edges[i*4+2]
            a: int = edges[i*4+3]
            add: bool = False
            if hole[0] == eb:
                # The segment matches the beginning of the hole boundary
                nhole = push_front(ea, hole, nhole)
                nhreg = push_front(r, hreg, nhreg)
                nharea = push_front(a, harea, nharea)
                add = True
            elif hole[nhole-1] == ea:
                # The segment matches the end of the hole boundary
                nhole = push_back(eb, hole, nhole)
                nhreg = push_back(r, hreg, nhreg)
                nharea = push_back(a, harea, nharea)
                add = True
            if add:
                # The edge segment was added, remove it
                edges[i*4+0] = edges[(nedges-1)*4+0]
                edges[i*4+1] = edges[(nedges-1)*4+1]
                edges[i*4+2] = edges[(nedges-1)*4+2]
                edges[i*4+3] = edges[(nedges-1)*4+3]
                nedges -= 1
                match = True
                i -= 1
            i += 1
        
        if not match:
            break

    tris: List[int] = [0] * (nhole*3)
    tverts: List[int] = [0] * (nhole*4)
    thole: List[int] = [0] * nhole
    
    # Generate temp vertex array for triangulation
    for i in range(nhole):
        pi: int = hole[i]
        tverts[i*4+0] = mesh.verts[pi*3+0]
        tverts[i*4+1] = mesh.verts[pi*3+1]
        tverts[i*4+2] = mesh.verts[pi*3+2]
        tverts[i*4+3] = 0
        thole[i] = i

    # Triangulate the hole
    ntris: int = triangulate(nhole, tverts, thole, tris)
    if ntris < 0:
        ntris = -ntris
        print("[Navmesh Baker] remove_vertex: triangulate() returned bad results")
   
    # Merge the hole triangles back to polygons
    polys: List[int] = [0] * ((ntris+1)*nvp)  # unsigned 2 bytes
    pregs: List[int] = [0] * ntris
    pareas: List[int] = [0] * ntris  # unsigned 1 byte
    tmpPoly: int = ntris*nvp  # pointer to polys
            
    # Build initial polygons
    npolys: int = 0
    for i in range(ntris*nvp):
        polys[i] = 0xffff
    for j in range(ntris):
        t: int = j*3  # pointer to tris
        if tris[t] != tris[t + 1] and tris[t] != tris[t + 2] and tris[t + 1] != tris[t + 2]:
            polys[npolys*nvp+0] = hole[tris[t]]
            polys[npolys*nvp+1] = hole[tris[t + 1]]
            polys[npolys*nvp+2] = hole[tris[t + 2]]

            # If this polygon covers multiple region types then
            # mark it as such
            if hreg[tris[t]] != hreg[tris[t + 1]] or hreg[tris[t + 1]] != hreg[tris[t + 2]]:
                pregs[npolys] = RC_MULTIPLE_REGS
            else:
                pregs[npolys] = hreg[tris[t]]

            pareas[npolys] = harea[tris[t]]
            npolys += 1
    if npolys == 0:
        return True
    
    # Merge polygons
    if nvp > 3:
        while True:
            # Find best polygons to merge
            bestMergeVal: int = 0
            bestPa: int = 0
            bestPb: int = 0
            bestEa: int = 0
            bestEb: int = 0
            
            for j in range(npolys - 1):
                pj: int = j*nvp  # pointer to polys
                for k in range(j + 1, npolys):
                    pk: int = k*nvp
                    ea = 0
                    eb = 0
                    v, ea, eb = get_poly_merge_value(polys, pj, pk, mesh.verts, ea, eb, nvp)
                    if v > bestMergeVal:
                        bestMergeVal = v
                        bestPa = j
                        bestPb = k
                        bestEa = ea
                        bestEb = eb
            
            if bestMergeVal > 0:
                # Found best, merge
                pa: int = bestPa*nvp  # pointer to polys
                pb: int = bestPb*nvp
                merge_poly_verts(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp)
                if pregs[bestPa] != pregs[bestPb]:
                    pregs[bestPa] = RC_MULTIPLE_REGS

                last: int = (npolys-1)*nvp  # pointer to polys
                if pb != last:
                    for m in range(nvp):
                        polys[pb + m] = polys[last + m]
                pregs[bestPb] = pregs[npolys-1]
                pareas[bestPb] = pareas[npolys-1]
                npolys -= 1
            else:
                # Could not merge any polygons, stop
                break
    
    # Store polygons
    for i in range(npolys):
        if mesh.npolys >= max_tris:
            break
        p = mesh.npolys*nvp*2  # pointer to mesh.polys
        for m in range(nvp * 2):
            mesh.polys[p + m] = 0xffff
        for j in range(nvp):
            mesh.polys[p + j] = polys[i*nvp+j]
        mesh.regs[mesh.npolys] = pregs[i]
        mesh.areas[mesh.npolys] = pareas[i]
        mesh.npolys += 1
        if mesh.npolys > max_tris:
            print("[Navmesh Baker]: remove_vertex: Too many polygons " + str(mesh.npolys) + " (max: " + str(max_tris) + ").")
            return False

    return True

def build_mesh_adjacency(polys: List[int],
                         npolys: int,
                         nverts: int,
                         verts_per_poly: int) -> bool:
    maxEdgeCount: int = npolys*verts_per_poly
    firstEdge: List[int] = [0] * (nverts + maxEdgeCount)  # unsigned 2 bytes
    nextEdge: int = nverts  # pointer to firstEdge
    edgeCount: int = 0
    
    edges: List[Edge] = [Edge() for _ in range(maxEdgeCount)]
    
    for i in range(nverts):
        firstEdge[i] = RC_MESH_NULL_IDX
    
    t: int = 0
    v0: int = 0
    v1: int = 0
    for i in range(npolys):
        t = i*verts_per_poly*2  # pointer to polys
        for j in range(verts_per_poly):
            if polys[t + j] == RC_MESH_NULL_IDX:
                break
            v0 = polys[t + j]
            v1 = polys[t] if (j + 1 >= verts_per_poly or polys[t + j + 1] == RC_MESH_NULL_IDX) else polys[t + j + 1]
            if v0 < v1:
                edge: Edge = edges[edgeCount]
                edge.vert[0] = v0
                edge.vert[1] = v1
                edge.poly[0] = i
                edge.poly_edge[0] = j
                edge.poly[1] = i
                edge.poly_edge[1] = 0
                # Insert edge
                firstEdge[nextEdge + edgeCount] = firstEdge[v0]
                firstEdge[v0] = edgeCount
                edgeCount += 1
    for i in range(npolys):
        t = i*verts_per_poly*2  # pointer to polys
        for j in range(verts_per_poly):
            if polys[t + j] == RC_MESH_NULL_IDX:
                break
            v0 = polys[t + j]
            v1 = polys[t] if (j + 1 >= verts_per_poly or polys[t + j + 1] == RC_MESH_NULL_IDX) else polys[t + j + 1]
            if v0 > v1:
                e: int = firstEdge[v1]
                while e != RC_MESH_NULL_IDX:
                    edge2: Edge = edges[e]
                    if edge2.vert[1] == v0 and edge2.poly[0] == edge2.poly[1]:
                        edge2.poly[1] = i
                        edge2.poly_edge[1] = j
                        break
                    e = firstEdge[nextEdge + e]

    # Store adjacency
    for i in range(edgeCount):
        edge_i: Edge = edges[i]
        if edge_i.poly[0] != edge_i.poly[1]:
            p0: int = edge_i.poly[0]*verts_per_poly*2  # pointer to polys
            p1: int = edge_i.poly[1]*verts_per_poly*2
            polys[p0 + verts_per_poly + edge_i.poly_edge[0]] = edge_i.poly[1]
            polys[p1 + verts_per_poly + edge_i.poly_edge[1]] = edge_i.poly[0]
    
    return True

def build_poly_mesh(cset: ContourSet,
                    nvp: int,
                    mesh: PolyMesh) -> bool:
    mesh.bmin = (cset.bmin[0], cset.bmin[1], cset.bmin[2])
    mesh.bmax = (cset.bmax[0], cset.bmax[1], cset.bmax[2])
    mesh.cs = cset.cs
    mesh.ch = cset.ch
    mesh.border_size = cset.border_size
    mesh.max_edge_error = cset.max_error

    max_vertices: int = 0
    max_tris: int = 0
    max_verts_per_cont: int = 0
    for i in range(cset.nconts):
        # Skip null contours
        if cset.conts[i].nverts < 3:
            continue
        max_vertices += cset.conts[i].nverts
        max_tris += cset.conts[i].nverts - 2
        max_verts_per_cont = max(max_verts_per_cont, cset.conts[i].nverts)
    
    if max_vertices >= 0xfffe:  # 65534
        print("[Navmesh Baker] build_poly_mesh: Too many vertices " + str(max_vertices))
        return False

    vflags: List[int] = [0] * max_vertices  # 1 byte per element
    mesh.verts = [0] * (max_vertices * 3)  # 2 bytes
    mesh.polys = [65535] * (max_tris * nvp * 2)  # 2 bytes
    mesh.regs = [0] * max_tris  # 2 bytes
    mesh.areas = [0] * max_tris  # 2 bytes

    mesh.nverts = 0
    mesh.npolys = 0
    mesh.nvp = nvp
    mesh.maxpolys = max_tris

    next_vert: List[int] = [0] * max_vertices
    first_vert: List[int] = [-1] * VERTEX_BUCKET_COUNT
    indices: List[int] = [0] * max_verts_per_cont
    tris: List[int] = [0] * (max_verts_per_cont * 3)
    polys: List[int] = [0] * ((max_verts_per_cont + 1) * nvp)  # 2 unsigned bytes
    tmp_poly: int = max_verts_per_cont * nvp  # pointer to polys array

    j: int = 0
    k: int = 0
    for i in range(cset.nconts):
        cont: Contour = cset.conts[i]
        
        # Skip null contours
        if cont.nverts < 3:
            continue
        
        # Triangulate contour
        for j in range(cont.nverts):
            indices[j] = j
            
        ntris: int = triangulate(cont.nverts, cont.verts, indices, tris)

        if ntris <= 0:
            # Bad triangulation, should not happen
            ntris = -ntris
                
        # Add and merge vertices
        for j in range(cont.nverts):
            indices[j], mesh.nverts = add_vertex(cont.verts[j*4], cont.verts[j*4 + 1], cont.verts[j*4 + 2], mesh.verts, first_vert, next_vert, mesh.nverts)
            if cont.verts[j*4 + 3] & RC_BORDER_VERTEX:
                # This vertex should be removed
                vflags[indices[j]] = 1

        # Build initial polygons
        npolys: int = 0
        for k in range(max_verts_per_cont * nvp):
            polys[k] = 0xffff  # 65535

        for j in range(ntris):
            if tris[j*3] != tris[j*3 + 1] and tris[j*3] != tris[j*3 + 2] and tris[j*3 + 1] != tris[j*3 + 2]:
                polys[npolys*nvp+0] = indices[tris[j*3]]
                polys[npolys*nvp+1] = indices[tris[j*3 + 1]]
                polys[npolys*nvp+2] = indices[tris[j*3 + 2]]
                npolys += 1
        if npolys == 0:
            continue

        # Merge polygons
        if nvp > 3:
            while True:
                # Find best polygons to merge
                bestMergeVal: int = 0
                bestPa: int = 0
                bestPb: int = 0
                bestEa: int = 0
                bestEb: int = 0
                
                j = 0
                while j < npolys - 1:
                    pj_ptr: int = j*nvp
                    k = j + 1
                    while k < npolys:
                        pk_ptr: int = k*nvp
                        ea: int = 0
                        eb: int = 0
                        (v, ea, eb) = get_poly_merge_value(polys, pj_ptr, pk_ptr, mesh.verts, ea, eb, nvp)
                        if v > bestMergeVal:
                            bestMergeVal = v
                            bestPa = j
                            bestPb = k
                            bestEa = ea
                            bestEb = eb
                        k += 1
                    j += 1
                
                if bestMergeVal > 0:
                    # Found best, merge
                    pa_ptr: int = bestPa*nvp
                    pb_ptr: int = bestPb*nvp
                    merge_poly_verts(polys, pa_ptr, pb_ptr, bestEa, bestEb, tmp_poly, nvp)
                    lastPoly_ptr: int = (npolys-1)*nvp
                    if pb_ptr != lastPoly_ptr:
                        for s in range(nvp):
                            polys[pb_ptr + s] = polys[lastPoly_ptr + s]
                    npolys -= 1
                else:
                    # Could not merge any polygons, stop
                    break
        
        # Store polygons
        for j in range(npolys):
            p_ptr: int = mesh.npolys*nvp*2  # pointer to mesh.polys
            q_ptr: int = j*nvp  # pointer to polys array
            for k in range(nvp):
                mesh.polys[p_ptr + k] = polys[q_ptr + k]
            mesh.regs[mesh.npolys] = cont.reg
            mesh.areas[mesh.npolys] = cont.area
            mesh.npolys += 1
            if mesh.npolys > max_tris:
                return False

    # Remove edge vertices
    i = 0
    while i < mesh.nverts:
        if vflags[i] > 0:
            if not can_remove_vertex(mesh, i):
                i += 1
                continue
            if not remove_vertex(mesh, i, max_tris):
                # Failed to remove vertex
                print("[Navmesh Baker] build_poly_mesh: Failed to remove edge vertex " + str(i))
                return False
            # Remove vertex
            # Note: mesh.nverts is already decremented inside removeVertex()!
            # Fixup vertex flags
            for j in range(i, mesh.nverts):
                vflags[j] = vflags[j+1]
            i -= 1
        i += 1
    
    # Calculate adjacency
    if not build_mesh_adjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp):
        print("[Navmesh Baker] build_poly_mesh: Adjacency failed")
        return False
    
    # Find portal edges
    if mesh.border_size > 0:
        w: int = cset.width
        h: int = cset.height
        for i in range(mesh.npolys):
            p: int = i*2*nvp  # pointer to mesh.polys
            for j in range(nvp):
                if mesh.polys[p + j] == RC_MESH_NULL_IDX:
                    break
                # Skip connected edges
                if mesh.polys[p + nvp+j] != RC_MESH_NULL_IDX:
                    continue
                nj: int = j+1
                if nj >= nvp or mesh.polys[p + nj] == RC_MESH_NULL_IDX:
                    nj = 0
                va: int = mesh.polys[p+j]*3  # pointer to mesh.verts
                vb: int = mesh.polys[p+nj]*3

                if mesh.verts[va] == 0 and mesh.verts[vb] == 0:
                    mesh.polys[p + nvp+j] = 0x8000 | 0
                elif mesh.verts[va + 2] == h and mesh.verts[vb + 2] == h:
                    mesh.polys[p + nvp+j] = 0x8000 | 1
                elif mesh.verts[va] == w and mesh.verts[vb] == w:
                    mesh.polys[p + nvp+j] = 0x8000 | 2
                elif mesh.verts[va + 2] == 0 and mesh.verts[vb + 2] == 0:
                    mesh.polys[p + nvp+j] = 0x8000 | 3
    mesh.flags = [0] * mesh.npolys

    if mesh.nverts > 0xffff:
        print("[Navmesh Baker] build_poly_mesh: The resulting mesh has too many vertices " + str(mesh.nverts) + " (max " + str(0xffff) + "). Data can be corrupted")
    if mesh.npolys > 0xffff:
        print("[Navmesh Baker] build_poly_mesh: The resulting mesh has too many polygons " + str(mesh.npolys) + " (max " + str(0xffff) + "). Data can be corrupted")

    return True
