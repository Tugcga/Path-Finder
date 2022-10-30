import { NavmeshNode,
    staticarray_navmeshnode_bytes_length, staticarray_navmeshnode_to_bytes, staticarray_navmeshnode_from_bytes_expr } from "./navmesh_node";
import { NavmeshBVH, TrianglesBVH } from "./navmesh_bvh";
import { Graph,
    staticarray_graph_bytes_length, staticarray_graph_to_bytes, staticarray_graph_from_bytes_expr } from "./navmesh_graph";
import { is_edge_new, squared_len, log_message } from "../common/utilities";
import { List } from "../common/list";

import { Serializable, SD_TYPE,
    i32_bytes_length, i32_to_bytes, i32_from_bytes,
    f32_bytes_length, f32_to_bytes, f32_from_bytes,
    bool_bytes_length, bool_to_bytes, bool_from_bytes,
    staticarray_f32_bytes_length, staticarray_f32_to_bytes, staticarray_f32_from_bytes_expr,
    staticarray_i32_bytes_length, staticarray_i32_to_bytes, staticarray_i32_from_bytes_expr,
    staticarray_staticarray_i32_bytes_length, staticarray_staticarray_i32_to_bytes, staticarray_staticarray_i32_from_bytes_expr } from "../common/binary_io";

export var NAVMESH_INITIAL_BUFFER_SIZE = 128;
export var BVH_AABB_DELTA: f32 = 0.5;

export class Navmesh extends Serializable {
    private m_vertices: StaticArray<f32>;
    private m_polygons: StaticArray<i32>;
    private m_sizes: StaticArray<i32>;
    private m_nodes_count: i32;

    private m_nodes: StaticArray<NavmeshNode>;
    private m_groups: StaticArray<StaticArray<i32>>;
    private m_groups_count: i32;
    private m_graphs: StaticArray<Graph>;

    private m_nodes_bvh: NavmeshBVH;
    private m_triangles_bvh: TrianglesBVH;

    private m_is_planar: bool;  // if true, then our navmesh is planar
    private m_planar_y: f32;  // this parameter contains y-value of the navmesh plane

    //buffers for path simplify process
    private b_portal_apex: StaticArray<f32>;
    private b_portal_left: StaticArray<f32>;
    private b_portal_right: StaticArray<f32>;
    private b_left: StaticArray<f32>;
    private b_right: StaticArray<f32>;
    private b_raw_path: StaticArray<f32>;
    private b_finall_path: StaticArray<f32>;
    private b_last_path_point: StaticArray<f32>;

    constructor(vertices: StaticArray<f32> = new StaticArray<f32>(0),
                polygons: StaticArray<i32> = new StaticArray<i32>(0),
                sizes: StaticArray<i32> = new StaticArray<i32>(0)) {
        super();
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
        this.b_last_path_point = new StaticArray<f32>(3);

        //generate nodes
        this.m_nodes = new StaticArray<NavmeshNode>(this.m_nodes_count);
        let vertex_map = new Map<i32, Array<i32>>();  // key - vertex index, value - array of polygon indexes with this corner
        let y_max = <f32>Number.MIN_VALUE;  // find the minimal and maximal value of the y-coordinate in tha navmesh vertices
        let y_min = <f32>Number.MAX_VALUE;
        //init vertex map by empty arrays
        for (let i = 0, len = vertices.length / 3; i < len; i++) {
            vertex_map.set(i, []);
            const y = unchecked(vertices[3*i + 1]);
            if(y < y_min){ y_min = y; }
            if(y > y_max){ y_max= y; }
        }
        if(Mathf.abs(y_max - y_min) < 0.0001){
            this.m_is_planar = true;
            this.m_planar_y = y_min;
        }
        else{
            this.m_is_planar = false;
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
                    let int0 = intersection[0];
                    if (int0 != node_index) {
                        node.add_neighbor(
                            unchecked(int0),
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
                } else {
                    //in fact this is impossible
                }
            }
        }

        //define groups
        let groups_queue = new Array<i32>(this.m_nodes.length);
        let groups_queue_length: i32 = 0;
        let t_groups = new Array<Array<i32>>();
        for (let i = 0, len = this.m_nodes.length; i < len; i++) {
            let node = unchecked(this.m_nodes[i]);
            let g = node.get_group();
            if (g == -1) {
                let new_group = new Array<i32>();
                let new_index = t_groups.length;
                //instead of recursive process, reassign groups for nodes by using task queue
                let is_finish: bool = false;
                unchecked(groups_queue[0] = node.m_index);
                groups_queue_length++;
                while(!is_finish){
                    if(groups_queue_length == 0){
                        is_finish = true;
                    }
                    else{
                        //get the last values in the queue
                        let ni = unchecked(groups_queue[groups_queue_length - 1]);
                        groups_queue_length -= 1;
                        //set the group for this node
                        let ni_node = unchecked(this.m_nodes[ni]);
                        let ni_node_neigh = ni_node.m_neighbor;
                        if(ni_node.m_group == -1){
                            ni_node.m_group = new_index;
                            new_group.push(ni);
                        }
                        //then iterate throw children
                        for(let k = 0, klen = ni_node.m_neighbor_count; k < klen; k++){
                            let knode = unchecked(this.m_nodes[ni_node_neigh[k]]);
                            let gk = knode.get_group();
                            if(gk == -1){
                                unchecked(groups_queue[groups_queue_length] = knode.m_index);
                                groups_queue_length++;
                            }
                        }
                    }
                }
                t_groups.push(new_group);
            }
        }
        //copy temp group to static arrays group
        this.m_groups_count = t_groups.length;
        let m_groups = new StaticArray<StaticArray<i32>>(this.m_groups_count);
        this.m_groups = m_groups;

        for (let i = 0, len = this.m_groups_count; i < len; i++) {
            let t_group = unchecked(t_groups[i]);
            let m_group = new StaticArray<i32>(t_group.length);
            unchecked(m_groups[i] = m_group);
            for (let j = 0, jlen = m_group.length; j < jlen; j++) {
                unchecked(m_group[j] = t_group[j]);
            }
        }

        //define one graph for each group
        this.m_graphs = new StaticArray<Graph>(this.m_groups_count);

        for (let i = 0, len = this.m_groups_count; i < len; i++) {
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
            unchecked(this.m_graphs[i] = new Graph(graph_verts_positions, graph_verts_s, graph_edges_s));
        }

        //and finally, buld bvh and triangles bvh
        this.m_nodes_bvh = new NavmeshBVH(this.m_nodes, BVH_AABB_DELTA);

        //form triangles
        shift = 0;
        let tr_index = 0;
        let triangles = new StaticArray<f32>(triangles_count * 9);  // because each triangle has 3 vertices (with 3 coordinates)
        for (let i = 0, len = sizes.length; i < len; i++) {
            let size = unchecked(sizes[i]);
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

    @inline
    get_is_planar(): bool{
        return this.m_is_planar;
    }

    @inline
    get_planar_y(): f32{
        return this.m_planar_y;
    }

    @inline
    get_groups_count(): i32{
        return this.m_groups_count;
    }

    @inline
    get_group_polygons(group_index: i32): StaticArray<StaticArray<i32>>{
        if(group_index < this.m_groups_count){
            let group = this.m_groups[group_index];
            let to_return = new StaticArray<StaticArray<i32>>(group.length);
            for(let i = 0, len = group.length; i < len; i++){
                to_return[i] = this.m_nodes[group[i]].get_polygon();
            }
            return to_return;
        }
        else{
            return new StaticArray<StaticArray<i32>>(0);
        }
    }

    @inline
    sample_polygon(position_x: f32, position_y: f32, position_z: f32): NavmeshNode | null{
        const node_index = this.m_nodes_bvh.sample(position_x, position_y, position_z);
        if(node_index >= 0){
            return this.m_nodes[node_index];
        }
        else{
            return null;
        }
    }

    search_path(s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): StaticArray<f32> {
        //generate the path between point (s_x, s_y, s_z) and (e_x, e_y, e_z)
        let start_index  = this.m_nodes_bvh.sample(s_x, s_y, s_z);
        let finish_index = this.m_nodes_bvh.sample(e_x, e_y, e_z);

        if (start_index == -1 || finish_index == -1) {
            return new StaticArray<f32>(0);  // points are not in navmesh, return empty path
        }

        if (start_index == finish_index) {
            let to_return = new StaticArray<f32>(6);
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

            // memorize last path point
            this.b_last_path_point[0] = this.b_finall_path[0];
            this.b_last_path_point[1] = this.b_finall_path[1];
            this.b_last_path_point[2] = this.b_finall_path[2];

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
                        if(!this._v_equal(this.b_portal_left, this.b_last_path_point)){
                            unchecked(this.b_finall_path[3 * finall_path_length + 0] = this.b_portal_left[0]);
                            unchecked(this.b_finall_path[3 * finall_path_length + 1] = this.b_portal_left[1]);
                            unchecked(this.b_finall_path[3 * finall_path_length + 2] = this.b_portal_left[2]);
                            unchecked(this.b_last_path_point[0] = this.b_portal_left[0]); unchecked(this.b_last_path_point[1] = this.b_portal_left[1]); unchecked(this.b_last_path_point[2] = this.b_portal_left[2]);
                            finall_path_length++;
                        }

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
                            unchecked(this.b_last_path_point[0] = this.b_portal_right[0]); unchecked(this.b_last_path_point[1] = this.b_portal_right[1]); unchecked(this.b_last_path_point[2] = this.b_portal_right[2]);
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
                unchecked(this.b_last_path_point[0] = this.b_right[0]); unchecked(this.b_last_path_point[1] = this.b_right[1]); unchecked(this.b_last_path_point[2] = this.b_right[2]);
                finall_path_length++;
            }

            //convert final path to output format
            let len = finall_path_length * 3;
            let to_return = new StaticArray<f32>(len);
            for (let k = 0; k < len; k++) {
                unchecked(to_return[k] = this.b_finall_path[k]);
            }
            return to_return;
        }
        return new StaticArray<f32>(0);
    }

    search_path_batch(coordinates: StaticArray<f32>): StaticArray<f32> {
        // coordinates is array with coordinates of start and end points
        // the length of the array should be <the number of pairs> x 6, because each pair defined by 6 float values (3 for start, 3 for end)
        let coordinates_length = coordinates.length;
        if(coordinates_length % 6 == 0) {
            let pairs_count: i32 = coordinates_length / 6;
            let to_return = new List<f32>();
            for(let step = 0; step < pairs_count; step++) {
                let step_path = this.search_path(unchecked(coordinates[6*step]), unchecked(coordinates[6*step + 1]), unchecked(coordinates[6*step + 2]),
                                                 unchecked(coordinates[6*step + 3]), unchecked(coordinates[6*step + 4]), unchecked(coordinates[6*step + 5]));
                to_return.push((step_path.length / 3) as f32); // add the number of points in the path
                // next add to the list actual points cooridnates
                for(let i = 0, len = step_path.length; i < len; i++) {
                    to_return.push(unchecked(step_path[i]));
                }
            }

            return to_return.to_static();
        } else {
            // invalid input array
            return new StaticArray<f32>(0);
        }
    }

    @inline
    sample(x: f32, y: f32, z: f32): StaticArray<f32> {
        return this.m_triangles_bvh.sample(x, y, z);
    }

    @inline
    private get_polygon_index(x: f32, y: f32, z: f32): i32{
        return this.m_nodes_bvh.sample(x, y, z);
    }

    private _get_nodes_group_index(index_01: i32, index_02: i32): i32 {
        let groups = this.m_groups;
        for (let i = 0, len = this.m_groups_count; i < len; i++) {
            let group = unchecked(groups[i]);
            if (group.includes(index_01) && group.includes(index_02)){
                return i;
            }
        }
        return -1;
    }

    @inline
    private _v_equal(a: StaticArray<f32>, b: StaticArray<f32>, epsilon: f32 = 0.0001): bool {
        return squared_len(
            unchecked(a[0] - b[0]),
            unchecked(a[1] - b[1]),
            unchecked(a[2] - b[2])
        ) < epsilon;
    }

    @inline
    private _triangle_area_2(a: StaticArray<f32>, b: StaticArray<f32>, c: StaticArray<f32>): f32 {
        return (
            unchecked(c[0] - a[0]) * unchecked(b[2] - a[2]) -
            unchecked(b[0] - a[0]) * unchecked(c[2] - a[2])
        );
    }

    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);

        view.setInt32(0, SD_TYPE.SD_TYPE_NAVMESH);
        view.setInt32(4, bytes_length);
        let shift = 8;

        to_return.set(staticarray_f32_to_bytes(this.m_vertices), shift);
        shift += staticarray_f32_bytes_length(this.m_vertices)

        to_return.set(staticarray_i32_to_bytes(this.m_polygons), shift);
        shift += staticarray_i32_bytes_length(this.m_polygons)

        to_return.set(staticarray_i32_to_bytes(this.m_sizes), shift);
        shift += staticarray_i32_bytes_length(this.m_sizes)

        to_return.set(i32_to_bytes(this.m_nodes_count), shift);
        shift += i32_bytes_length();

        to_return.set(staticarray_navmeshnode_to_bytes(this.m_nodes), shift);
        shift += staticarray_navmeshnode_bytes_length(this.m_nodes);

        to_return.set(staticarray_staticarray_i32_to_bytes(this.m_groups), shift);
        shift += staticarray_staticarray_i32_bytes_length(this.m_groups);

        to_return.set(i32_to_bytes(this.m_groups_count), shift);
        shift += i32_bytes_length();

        to_return.set(staticarray_graph_to_bytes(this.m_graphs), shift);
        shift += staticarray_graph_bytes_length(this.m_graphs);

        to_return.set(this.m_nodes_bvh.to_bytes(), shift);
        shift += this.m_nodes_bvh.bytes_length();

        to_return.set(this.m_triangles_bvh.to_bytes(), shift);
        shift += this.m_triangles_bvh.bytes_length();

        to_return.set(bool_to_bytes(this.m_is_planar), shift);
        shift += bool_bytes_length();

        to_return.set(f32_to_bytes(this.m_planar_y), shift);
        shift += f32_bytes_length();

        return to_return;
    }

    override from_bytes(bytes: Uint8Array): void {
        if(bytes.length > 0) {
            let view = new DataView(bytes.buffer);
            const id = view.getInt32(0);
            let shift = 0;
            if(id == SD_TYPE.SD_TYPE_NAVMESH) {
                shift = 8;
            } else { return; }

            const m_vertices_id = view.getInt32(shift);
            if(m_vertices_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
                const bl = view.getInt32(shift + 4);
                this.m_vertices = staticarray_f32_from_bytes_expr(view, shift);
                shift += bl;
            } else { return; }

            const m_polygons_id = view.getInt32(shift);
            if(m_polygons_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
                const bl = view.getInt32(shift + 4);
                this.m_polygons = staticarray_i32_from_bytes_expr(view, shift);
                shift += bl;
            } else { return; }

            const m_sizes_id = view.getInt32(shift);
            if(m_sizes_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
                const bl = view.getInt32(shift + 4);
                this.m_sizes = staticarray_i32_from_bytes_expr(view, shift);
                shift += bl;
            } else { return; }

            const m_nodes_count_id = view.getInt32(shift);
            if(m_nodes_count_id == SD_TYPE.SD_TYPE_INT32) {
                const bl = view.getInt32(shift + 4);
                this.m_nodes_count = view.getInt32(shift + 8);
                shift += bl;
            } else { return; }

            const m_nodes_id = view.getInt32(shift);
            if(m_nodes_id == SD_TYPE.SD_TYPE_STATICARRAY_NAVMESHNODE) {
                const bl = view.getInt32(shift + 4);
                this.m_nodes = staticarray_navmeshnode_from_bytes_expr(view, shift);
                shift += bl;
            } else { return; }

            const m_groups_id = view.getInt32(shift);
            if(m_groups_id == SD_TYPE.SD_TYPE_STATICARRAY_STATICARRAY_INT32) {
                const bl = view.getInt32(shift + 4);
                this.m_groups = staticarray_staticarray_i32_from_bytes_expr(view, shift);
                shift += bl;
            } else { return; }

            const m_groups_count_id = view.getInt32(shift);
            if(m_groups_count_id == SD_TYPE.SD_TYPE_INT32) {
                const bl = view.getInt32(shift + 4);
                this.m_groups_count = view.getInt32(shift + 8);
                shift += bl;
            } else { return; }

            const m_graphs_id = view.getInt32(shift);
            if(m_graphs_id == SD_TYPE.SD_TYPE_STATICARRAY_GRAPH) {
                const bl = view.getInt32(shift + 4);
                this.m_graphs = staticarray_graph_from_bytes_expr(view, shift);
                shift += bl;
            } else { return; }

            const m_nodes_bvh_id = view.getInt32(shift);
            if(m_nodes_bvh_id == SD_TYPE.SD_TYPE_NAVMESHBVH) {
                const bl = view.getInt32(shift + 4);
                this.m_nodes_bvh = new NavmeshBVH();
                this.m_nodes_bvh.from_bytes(view, shift, this.m_nodes);
                shift += bl;
            } else { return; }

            const m_triangles_bvh_id = view.getInt32(shift);
            if(m_triangles_bvh_id == SD_TYPE.SD_TYPE_TRIANGLESBVH) {
                const bl = view.getInt32(shift + 4);
                this.m_triangles_bvh = new TrianglesBVH();
                this.m_triangles_bvh.from_bytes(view, shift);
                shift += bl;
            } else { return; }

            const m_is_planar_id = view.getInt32(shift);
            if(m_is_planar_id == SD_TYPE.SD_TYPE_BOOL) {
                const bl = view.getInt32(shift + 4);
                this.m_is_planar = view.getUint8(shift + 8) == 1;
                shift += bl;
            } else { return; }

            const m_planar_y_id = view.getInt32(shift);
            if(m_planar_y_id == SD_TYPE.SD_TYPE_FLOAT32) {
                const bl = view.getInt32(shift + 4);
                this.m_planar_y = view.getFloat32(shift + 8);
                shift += bl;
            } else { return; }
        }
    }

    override bytes_length(): u32 {
        let to_return = 8; // id, bytes length

        to_return += staticarray_f32_bytes_length(this.m_vertices);
        to_return += staticarray_i32_bytes_length(this.m_polygons);
        to_return += staticarray_i32_bytes_length(this.m_sizes);
        to_return += i32_bytes_length();  // m_nodes_count
        to_return += staticarray_navmeshnode_bytes_length(this.m_nodes);
        to_return += staticarray_staticarray_i32_bytes_length(this.m_groups);
        to_return += i32_bytes_length();  // m_groups_count
        to_return += staticarray_graph_bytes_length(this.m_graphs);
        to_return += this.m_nodes_bvh.bytes_length();
        to_return += this.m_triangles_bvh.bytes_length();

        to_return += bool_bytes_length();  // m_is_planar
        to_return += f32_bytes_length();  // m_planar_y

        return to_return;
    }
}

export function set_bvh_delta(delta: f32): void {
    BVH_AABB_DELTA = delta;
}

export function get_bvh_delta(): f32 {
    return BVH_AABB_DELTA;
}
