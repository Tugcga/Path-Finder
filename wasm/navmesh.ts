import {NavmeshNode} from "./navmesh_node";
import {INavmeshBVH, ITrianglesBVH} from "./navmesh_bvh";
import {INavmeshGraph} from "./navmesh_graph";

namespace console {
    declare function log(str: string): void;
}

export const Float32Array_ID = idof<Float32Array>();
export const Int32Array_ID = idof<Int32Array>();

export var NAVMESH_INITIAL_BUFFER_SIZE = 128;
export var BVH_AABB_DELTA: f32 = 0.5;

@inline
function squared_len(x: f32, y: f32, z: f32): f32 {
    return x * x + y * y + z * z;
}

function is_edge_new(edges: Array<i32>, a: i32, b: i32): bool {
    for (let i = 0, len = edges.length / 2; i < len; i++) {
        let e0 = unchecked(edges[2 * i + 0]);
        let e1 = unchecked(edges[2 * i + 1]);
        if ((e0 == a && e1 == b) || (e0 == b && e1 == a)) {
            return false;
        }
    }
    return true;
}

export class NavmeshBVH extends INavmeshBVH {

}

export class TrianglesBVH extends ITrianglesBVH {

}

export class NavmeshGraph extends INavmeshGraph {

}

export class Navmesh{
    m_vertices: StaticArray<f32>;
    m_polygons: StaticArray<i32>;
    m_sizes: StaticArray<i32>;
    m_nodes_count: i32;

    m_nodes: StaticArray<NavmeshNode>;
    m_groups: StaticArray<StaticArray<i32>>;
    m_groups_count: i32;
    m_graphs: StaticArray<NavmeshGraph>;

    m_nodes_bvh: NavmeshBVH;
    m_triangles_bvh: TrianglesBVH;

    //buffers for path simplify process
    b_portal_apex: StaticArray<f32>;
    b_portal_left: StaticArray<f32>;
    b_portal_right: StaticArray<f32>;
    b_left: StaticArray<f32>;
    b_right: StaticArray<f32>;
    b_raw_path: StaticArray<f32>;
    b_finall_path: StaticArray<f32>;

    constructor(vertices: StaticArray<f32>, polygons: StaticArray<i32>, sizes: StaticArray<i32>) {
        this.m_vertices = vertices;
        this.m_polygons = polygons;
        this.m_sizes = sizes;

        this.m_nodes_count = sizes.length;

        this.b_portal_apex = new StaticArray<f32>(3);
        this.b_portal_left = new StaticArray<f32>(3);
        this.b_portal_right = new StaticArray<f32>(3);
        this.b_left = new StaticArray<f32>(3);
        this.b_right = new StaticArray<f32>(3);

        this.b_raw_path = new StaticArray<f32>(NAVMESH_INITIAL_BUFFER_SIZE);
        this.b_finall_path = new StaticArray<f32>(NAVMESH_INITIAL_BUFFER_SIZE);

        //generate nodes
        this.m_nodes = new StaticArray<NavmeshNode>(this.m_nodes_count);
        let vertex_map = new Map<i32, Array<i32>>();  // key - vertex index, value - array of polygon indexes with this corner
        //init vertex map by empty arrays
        for (let i = 0, len = vertices.length / 3; i < len; i++) {
            vertex_map.set(i, []);
        }
        let shift = 0;
        let triangles_count = 0;
        let nodes_count = this.m_nodes_count;
        for (let i = 0; i < nodes_count; i++) {
            let size = unchecked(sizes[i]);
            triangles_count += size - 2;
            let polygon_indexes = new StaticArray<i32>(size);
            for (let j = 0, len = polygon_indexes.length; j < len; j++) {
                let v = unchecked(polygons[shift]);
                unchecked(polygon_indexes[j] = v);
                //add polygon to the vertex map
                unchecked(vertex_map[v]).push(i);
                shift++;
            }

            unchecked(this.m_nodes[i] = new NavmeshNode(this.m_vertices, i, polygon_indexes));
        }

        // define neighborhoods
        for (let node_index = 0; node_index < nodes_count; node_index++) {
            let node = unchecked(this.m_nodes[node_index]);
            let node_vertices = node.get_vertex_indexes();
            let node_vertices_len = node_vertices.length;
            for (let i = 0; i < node_vertices_len; i++) {
                let j = i + 1;
                if (j == node_vertices_len) {
                    j = 0;
                }

                let u = unchecked(node_vertices[i]);
                let v = unchecked(node_vertices[j]);
                //manualy find intersection of two arrays
                let intersection = new Array<i32>(0);
                let array_a = vertex_map.get(u);
                let array_b = vertex_map.get(v);
                for (let i = 0, len_a = array_a.length; i < len_a; i++) {
                    let v = unchecked(array_a[i]);
                    if (array_b.includes(v)) {
                        intersection.push(v);
                    }
                }
                if (intersection.length == 2) {  // in all cases the intersection is 2
                    if (intersection[0] != node_index) {
                        node.add_neighbor(
                            unchecked(intersection[0]),
                            unchecked(this.m_vertices[3 * u + 0]),
                            unchecked(this.m_vertices[3 * u + 1]),
                            unchecked(this.m_vertices[3 * u + 2]),
                            unchecked(this.m_vertices[3 * v + 0]),
                            unchecked(this.m_vertices[3 * v + 1]),
                            unchecked(this.m_vertices[3 * v + 2])
                        );
                    } else {
                        node.add_neighbor(
                            unchecked(intersection[1]),
                            unchecked(this.m_vertices[3 * u + 0]),
                            unchecked(this.m_vertices[3 * u + 1]),
                            unchecked(this.m_vertices[3 * u + 2]),
                            unchecked(this.m_vertices[3 * v + 0]),
                            unchecked(this.m_vertices[3 * v + 1]),
                            unchecked(this.m_vertices[3 * v + 2])
                        );
                    }
                }
                else{
                    //in fact this is impossible
                }
            }
        }

        //define groups
        let t_groups = new Array<Array<i32>>();
        for (let i = 0, len = this.m_nodes.length; i < len; i++) {
            let node = unchecked(this.m_nodes[i]);
            let g = node.get_group();
            if (g == -1) {
                let new_group = new Array<i32>();
                let new_index = t_groups.length;
                node.set_group(new_index, new_group, this.m_nodes);
                t_groups.push(new_group);
            }
        }
        //copy temp group to static arrays group
        this.m_groups_count = t_groups.length;
        let m_groups = new StaticArray<StaticArray<i32>>(this.m_groups_count);
        this.m_groups = m_groups;

        for (let i = 0; i < this.m_groups_count; i++) {
            let t_group = unchecked(t_groups[i]);
            let m_group = new StaticArray<i32>(t_group.length);
            unchecked(m_groups[i] = m_group);
            for (let j = 0; j < m_group.length; j++) {
                unchecked(m_group[j] = t_group[j]);
            }
        }

        //define one graph for each group
        this.m_graphs = new StaticArray<NavmeshGraph>(this.m_groups_count);

        for (let i = 0; i < this.m_groups_count; i++) {
            let group = unchecked(this.m_groups[i]);
            let graph_edges = new Array<i32>();

            for (let j = 0, jlen = group.length; j < jlen; j++) {
                let node_index = unchecked(group[j]);
                let node_neighbors = unchecked(this.m_nodes[node_index]).get_neighbord();
                for (let k = 0, klen = node_neighbors.length; k < klen; k++) {
                    let other_node = unchecked(node_neighbors[k]);
                    if (is_edge_new(graph_edges, node_index, other_node)) {
                        if (node_index < other_node) {
                            graph_edges.push(node_index);
                            graph_edges.push(other_node);
                        } else {
                            graph_edges.push(other_node);
                            graph_edges.push(node_index);
                        }
                    }
                }
            }

            //make graph edges static array
            let graph_edges_len = graph_edges.length;
            let graph_edges_s = new StaticArray<i32>(graph_edges_len);
            for (let j = 0; j < graph_edges_len; j++) {
                unchecked(graph_edges_s[j] = graph_edges[j]);
            }

            //next graph vertex positions
            let graph_verts = new Array<i32>();
            for (let j = 0; j < graph_edges_len; j++) {
                let v = unchecked(graph_edges_s[j]);
                if (!graph_verts.includes(v)) {
                    graph_verts.push(v);
                }
            }
            //sort graph_verts
            graph_verts.sort();

            //make vertices static array
            let graph_verts_len = graph_verts.length;
            let graph_verts_s = new StaticArray<i32>(graph_verts_len);
            for (let j = 0; j < graph_verts_len; j++) {
                unchecked(graph_verts_s[j] = graph_verts[j]);
            }

            //caluclate positions of the vertices
            let graph_verts_positions = new StaticArray<f32>(3 * graph_verts_len);
            for (let j = 0; j < graph_verts_len; j++) {
                let v = unchecked(graph_verts_s[j]);
                let c = unchecked(this.m_nodes[v]).get_center();
                unchecked(graph_verts_positions[3 * j + 0] = c[0]);
                unchecked(graph_verts_positions[3 * j + 1] = c[1]);
                unchecked(graph_verts_positions[3 * j + 2] = c[2]);
            }

            //create the graph
            unchecked(this.m_graphs[i] = new NavmeshGraph(graph_verts_positions, graph_verts_s, graph_edges_s));
        }

        //and finally, buld bvh and triangles bvh
        this.m_nodes_bvh = new NavmeshBVH(this.m_nodes, BVH_AABB_DELTA);

        //form triangles
        shift = 0;
        let tr_index = 0;
        let triangles = new StaticArray<f32>(triangles_count * 9);  // because each triangle has 3 vertices (with 3 coordinates)
        for (let i = 0, len = sizes.length; i < len; i++) {
            let size = sizes[i];
            for (let j = 2; j < size; j++) {
                //add point 0, j - 1 and j
                let i1 = unchecked(polygons[shift]);
                let i2 = unchecked(polygons[shift + j - 1]);
                let i3 = unchecked(polygons[shift + j]);

                unchecked(triangles[9 * tr_index + 0] = vertices[3 * i1 + 0]);
                unchecked(triangles[9 * tr_index + 1] = vertices[3 * i1 + 1]);
                unchecked(triangles[9 * tr_index + 2] = vertices[3 * i1 + 2]);

                unchecked(triangles[9 * tr_index + 3] = vertices[3 * i2 + 0]);
                unchecked(triangles[9 * tr_index + 4] = vertices[3 * i2 + 1]);
                unchecked(triangles[9 * tr_index + 5] = vertices[3 * i2 + 2]);

                unchecked(triangles[9 * tr_index + 6] = vertices[3 * i3 + 0]);
                unchecked(triangles[9 * tr_index + 7] = vertices[3 * i3 + 1]);
                unchecked(triangles[9 * tr_index + 8] = vertices[3 * i3 + 2]);

                tr_index++;
            }
            shift += size;
        }

        this.m_triangles_bvh = new TrianglesBVH(triangles, BVH_AABB_DELTA);
    }

    search_path(s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): Float32Array {
        //generate the path between point (s_x, s_y, s_z) and (e_x, e_y, e_z)
        let start_index  = this.m_nodes_bvh.sample(s_x, s_y, s_z);
        let finish_index = this.m_nodes_bvh.sample(e_x, e_y, e_z);

        if (start_index == -1 || finish_index == -1) {
            return new Float32Array(0);  // points are not in navmesh, return empty path
        }

        if (start_index == finish_index) {
            let to_return = new Float32Array(6);
            unchecked(to_return[0] = s_x);
            unchecked(to_return[1] = s_y);
            unchecked(to_return[2] = s_z);
            unchecked(to_return[3] = e_x);
            unchecked(to_return[4] = e_y);
            unchecked(to_return[5] = e_z);
            return to_return;
        }

        let group_index = this._get_nodes_group_index(start_index, finish_index);
        if (group_index > -1) {
            let graph = unchecked(this.m_graphs[group_index]);
            let graph_path = graph.search(start_index, finish_index);

            let graph_path_edges_count = graph_path.length - 1;
            if (graph_path_edges_count < 0) {
                graph_path_edges_count = 0;
            }
            let raw_path_length = 3 * 4 + (3 * 2) * graph_path_edges_count;
            if (this.b_raw_path.length < raw_path_length) {
                this.b_raw_path    = new StaticArray<f32>(raw_path_length + NAVMESH_INITIAL_BUFFER_SIZE);
                this.b_finall_path = new StaticArray<f32>(raw_path_length + NAVMESH_INITIAL_BUFFER_SIZE);
            }
            unchecked(this.b_raw_path[0] = s_x);
            unchecked(this.b_raw_path[1] = s_y);
            unchecked(this.b_raw_path[2] = s_z);
            unchecked(this.b_raw_path[3] = s_x);
            unchecked(this.b_raw_path[4] = s_y);
            unchecked(this.b_raw_path[5] = s_z);

            for (let p_i = 0; p_i < graph_path_edges_count; p_i++) {
                let v0 = unchecked(graph_path[p_i + 0]);
                let v1 = unchecked(graph_path[p_i + 1]);
                let portal = unchecked(this.m_nodes[v0]).get_portal(v1);
                //add values to the raw_path
                unchecked(this.b_raw_path[(p_i + 1) * 6 + 0] = portal[0]);
                unchecked(this.b_raw_path[(p_i + 1) * 6 + 1] = portal[1]);
                unchecked(this.b_raw_path[(p_i + 1) * 6 + 2] = portal[2]);

                unchecked(this.b_raw_path[(p_i + 1) * 6 + 3] = portal[3]);
                unchecked(this.b_raw_path[(p_i + 1) * 6 + 4] = portal[4]);
                unchecked(this.b_raw_path[(p_i + 1) * 6 + 5] = portal[5]);
            }

            //add finish points
            unchecked(this.b_raw_path[raw_path_length - 6] = e_x);
            unchecked(this.b_raw_path[raw_path_length - 5] = e_y);
            unchecked(this.b_raw_path[raw_path_length - 4] = e_z);
            unchecked(this.b_raw_path[raw_path_length - 3] = e_x);
            unchecked(this.b_raw_path[raw_path_length - 2] = e_y);
            unchecked(this.b_raw_path[raw_path_length - 1] = e_z);

            unchecked(this.b_portal_apex[0] = this.b_raw_path[0]);
            unchecked(this.b_portal_apex[1] = this.b_raw_path[1]);
            unchecked(this.b_portal_apex[2] = this.b_raw_path[2]);

            unchecked(this.b_portal_left[0] = this.b_raw_path[0]);
            unchecked(this.b_portal_left[1] = this.b_raw_path[1]);
            unchecked(this.b_portal_left[2] = this.b_raw_path[2]);

            unchecked(this.b_portal_right[0] = this.b_raw_path[3]);
            unchecked(this.b_portal_right[1] = this.b_raw_path[4]);
            unchecked(this.b_portal_right[2] = this.b_raw_path[5]);

            let apex_index  = 0;
            let left_index  = 0;
            let right_index = 0;

            //crate buffer for the finall path
            unchecked(this.b_finall_path[0] = this.b_portal_apex[0]);
            unchecked(this.b_finall_path[1] = this.b_portal_apex[1]);
            unchecked(this.b_finall_path[2] = this.b_portal_apex[2]);

            let finall_path_length = 1;
            let i = 1;

            while (i < raw_path_length / 6) {
                unchecked(this.b_left[0] = this.b_raw_path[6 * i + 0]);
                unchecked(this.b_left[1] = this.b_raw_path[6 * i + 1]);
                unchecked(this.b_left[2] = this.b_raw_path[6 * i + 2]);

                unchecked(this.b_right[0] = this.b_raw_path[3 * (2 * i + 1) + 0]);
                unchecked(this.b_right[1] = this.b_raw_path[3 * (2 * i + 1) + 1]);
                unchecked(this.b_right[2] = this.b_raw_path[3 * (2 * i + 1) + 2]);

                let skip_next = false;

                if (this._triangle_area_2(this.b_portal_apex, this.b_portal_right, this.b_right) <= 0.0) {
                    if (
                        this._v_equal(this.b_portal_apex, this.b_portal_right) ||
                        this._triangle_area_2(this.b_portal_apex, this.b_portal_left, this.b_right) > 0.0
                    ) {
                        unchecked(this.b_portal_right[0] = this.b_right[0]);
                        unchecked(this.b_portal_right[1] = this.b_right[1]);
                        unchecked(this.b_portal_right[2] = this.b_right[2]);
                        right_index = i;
                    } else {
                        unchecked(this.b_finall_path[3 * finall_path_length + 0] = this.b_portal_left[0]);
                        unchecked(this.b_finall_path[3 * finall_path_length + 1] = this.b_portal_left[1]);
                        unchecked(this.b_finall_path[3 * finall_path_length + 2] = this.b_portal_left[2]);
                        finall_path_length++;

                        unchecked(this.b_portal_apex[0]  = this.b_portal_left[0]);
                        unchecked(this.b_portal_apex[1]  = this.b_portal_left[1]);
                        unchecked(this.b_portal_apex[2]  = this.b_portal_left[2]);
                        unchecked(this.b_portal_left[0]  = this.b_portal_apex[0]);
                        unchecked(this.b_portal_left[1]  = this.b_portal_apex[1]);
                        unchecked(this.b_portal_left[2]  = this.b_portal_apex[2]);
                        unchecked(this.b_portal_right[0] = this.b_portal_apex[0]);
                        unchecked(this.b_portal_right[1] = this.b_portal_apex[1]);
                        unchecked(this.b_portal_right[2] = this.b_portal_apex[2]);

                        apex_index  = left_index;
                        left_index  = apex_index;
                        right_index = apex_index;
                        i = apex_index;
                        skip_next = true;
                    }
                }
                if (!skip_next) {
                    if (this._triangle_area_2(this.b_portal_apex, this.b_portal_left, this.b_left) >= 0.0) {
                        if (
                            this._v_equal(this.b_portal_apex, this.b_portal_left) ||
                            this._triangle_area_2(this.b_portal_apex, this.b_portal_right, this.b_left) < 0.0
                        ) {
                            unchecked(this.b_portal_left[0] = this.b_left[0]);
                            unchecked(this.b_portal_left[1] = this.b_left[1]);
                            unchecked(this.b_portal_left[2] = this.b_left[2]);
                            left_index = i;
                        } else {
                            unchecked(this.b_finall_path[3 * finall_path_length + 0] = this.b_portal_right[0]);
                            unchecked(this.b_finall_path[3 * finall_path_length + 1] = this.b_portal_right[1]);
                            unchecked(this.b_finall_path[3 * finall_path_length + 2] = this.b_portal_right[2]);
                            finall_path_length++;

                            unchecked(this.b_portal_apex[0]  = this.b_portal_right[0]);
                            unchecked(this.b_portal_apex[1]  = this.b_portal_right[1]);
                            unchecked(this.b_portal_apex[2]  = this.b_portal_right[2]);
                            unchecked(this.b_portal_left[0]  = this.b_portal_apex[0]);
                            unchecked(this.b_portal_left[1]  = this.b_portal_apex[1]);
                            unchecked(this.b_portal_left[2]  = this.b_portal_apex[2]);
                            unchecked(this.b_portal_right[0] = this.b_portal_apex[0]);
                            unchecked(this.b_portal_right[1] = this.b_portal_apex[1]);
                            unchecked(this.b_portal_right[2] = this.b_portal_apex[2]);

                            apex_index  = right_index;
                            left_index  = apex_index;
                            right_index = apex_index;
                            i = apex_index;
                        }
                    }
                }
                i++;
            }

            unchecked(this.b_left[0] = this.b_finall_path[3 * (finall_path_length - 1) + 0]);
            unchecked(this.b_left[1] = this.b_finall_path[3 * (finall_path_length - 1) + 1]);
            unchecked(this.b_left[2] = this.b_finall_path[3 * (finall_path_length - 1) + 2]);

            unchecked(this.b_right[0] = this.b_raw_path[raw_path_length - 6]);
            unchecked(this.b_right[1] = this.b_raw_path[raw_path_length - 5]);
            unchecked(this.b_right[2] = this.b_raw_path[raw_path_length - 4]);

            if (!this._v_equal(this.b_left, this.b_right)) {
                unchecked(this.b_finall_path[3 * finall_path_length + 0] = this.b_right[0]);
                unchecked(this.b_finall_path[3 * finall_path_length + 1] = this.b_right[1]);
                unchecked(this.b_finall_path[3 * finall_path_length + 2] = this.b_right[2]);
                finall_path_length++;
            }

            //convert final path to output format
            let len = finall_path_length * 3;
            let to_return = new Float32Array(len);
            for (let k = 0; k < len; k++) {
                unchecked(to_return[k] = this.b_finall_path[k]);
            }
            return to_return;
        }
        return new Float32Array(0);
    }

    @inline
    sample(x: f32, y: f32, z: f32): Float32Array {
        return this.m_triangles_bvh.sample(x, y, z);
    }

    @inline
    get_polygon_index(x: f32, y: f32, z: f32): i32{
        return this.m_nodes_bvh.sample(x, y, z);
    }

    _get_nodes_group_index(index_01: i32, index_02: i32): i32 {
        for (let i = 0, len = this.m_groups_count; i < len; i++) {
            let group = unchecked(this.m_groups[i]);
            if (group.includes(index_01) && group.includes(index_02)){
                return i;
            }
        }
        return -1;
    }

    @inline
    _v_equal(a: StaticArray<f32>, b: StaticArray<f32>, epsilon: f32 = 0.01): bool {
        return squared_len(
            unchecked(a[0] - b[0]),
            unchecked(a[1] - b[1]),
            unchecked(a[2] - b[2])
        ) < epsilon;
    }

    @inline
    _triangle_area_2(a: StaticArray<f32>, b: StaticArray<f32>, c: StaticArray<f32>): f32 {
        return (
            unchecked(c[0] - a[0]) * unchecked(b[2] - a[2]) -
            unchecked(b[0] - a[0]) * unchecked(c[2] - a[2])
        );
    }
}

export function create_navmesh(vertices: Float32Array, polygons: Int32Array, sizes: Int32Array): Navmesh {
    let st_vertices = new StaticArray<f32>(vertices.length);
    let st_polygons = new StaticArray<i32>(polygons.length);
    let st_sizes = new StaticArray<i32>(sizes.length);

    for (let i = 0, len = vertices.length; i < len; i++) {
        unchecked(st_vertices[i] = vertices[i]);
    }

    for (let i = 0, len = polygons.length; i < len; i++) {
        unchecked(st_polygons[i] = polygons[i]);
    }

    for (let i = 0, len = sizes.length; i < len; i++) {
        unchecked(st_sizes[i] = sizes[i]);
    }

    return new Navmesh(st_vertices, st_polygons, st_sizes);
}

export function create_triangle_bvh(vertices: Float32Array): TrianglesBVH {
    //convert Float32Array to Static array
    let len = vertices.length;
    let verts = new StaticArray<f32>(len);
    for (let i = 0; i < len; i++) {
        unchecked(verts[i] = vertices[i]);
    }

    return new TrianglesBVH(verts, BVH_AABB_DELTA);
}

export function create_bvh(vertices: Float32Array, polygons: Int32Array, sizes: Int32Array): NavmeshBVH {
    //convert vertices to StaticArray
    let verts = new StaticArray<f32>(vertices.length);
    for (let i = 0, len = vertices.length; i < len; i++) {
        unchecked(verts[i] = vertices[i]);
    }
    //we should convert raw arrays to the list of nodes
    let shift = 0;
    let nodes = new StaticArray<NavmeshNode>(sizes.length);
    for (let i = 0, len = sizes.length; i < len; i++) {
        //form array of node vertices
        let polygon = new StaticArray<i32>(unchecked(sizes[i]));
        for (let j = 0; j < polygon.length; j++) {
            unchecked(polygon[j] = polygons[shift]);
            shift++;
        }
        //create node
        unchecked(nodes[i] = new NavmeshNode(verts, i, polygon));
    }

    return new NavmeshBVH(nodes, BVH_AABB_DELTA);
}

export function create_navmesh_graph(vertex_positions: Float32Array, vertices: Int32Array, edges: Int32Array): NavmeshGraph {
    //save positions
    let m_positions = new StaticArray<f32>(vertex_positions.length);
    for (let i = 0, len = vertex_positions.length; i < len; i++) {
        unchecked(m_positions[i] = vertex_positions[i]);
    }

    //save names
    let m_vertex_names = new StaticArray<i32>(vertices.length);
    for (let i = 0, len = vertices.length; i < len; i++) {
        unchecked(m_vertex_names[i] = vertices[i]);
    }

    //save edges
    let m_edges = new StaticArray<i32>(edges.length);
    for (let i = 0, len = edges.length; i < len; i++) {
        unchecked(m_edges[i] = edges[i]);
    }
    return new NavmeshGraph(m_positions, m_vertex_names, m_edges);
}

export function set_bvh_delta(delta: f32): void {
    BVH_AABB_DELTA = delta;
}

export function get_bvh_delta(): f32 {
    return BVH_AABB_DELTA;
}

export function set_navmesh_initial_buffer_size(value: i32): void {
    NAVMESH_INITIAL_BUFFER_SIZE = value;
}

export function get_navmesh_initial_buffer_size(): i32 {
    return NAVMESH_INITIAL_BUFFER_SIZE;
}
