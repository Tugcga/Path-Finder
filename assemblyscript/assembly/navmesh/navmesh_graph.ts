import { distance, log_message } from "../common/utilities";
import { Serializable, SD_TYPE, 
    i32_bytes_length, i32_to_bytes, i32_from_bytes,
    staticarray_f32_bytes_length, staticarray_f32_to_bytes, staticarray_f32_from_bytes_expr,
    staticarray_i32_bytes_length, staticarray_i32_to_bytes, staticarray_i32_from_bytes_expr,
    staticarray_bool_bytes_length, staticarray_bool_to_bytes, staticarray_bool_from_bytes,
    map_i32_i32_bytes_length, map_i32_i32_to_bytes, map_i32_i32_from_bytes_expr,
    map_i32_staticarray_i32_bytes_length, map_i32_staticarray_i32_to_bytes, map_i32_staticarray_i32_from_bytes_expr } from "../common/binary_io";

export class Graph extends Serializable {
    private m_positions: StaticArray<f32>;  // store by triples [x1, y1, z1, x2, y2, z2, ...]
    private m_vertex_names: StaticArray<i32>;  // in the same order as positions
    private m_vertex_count: i32;
    private m_edges: StaticArray<i32>;  // store by pairs [s1, e1, s2, e2, ...]
    private m_index_map: Map<i32, i32>;
    private m_incident_map: Map<i32, StaticArray<i32>>; // key - vertex, value - incident vertices (not names, but indices from 0 to n-1)

    //inner arrays for search algorithm
    private i_vertices_g: StaticArray<f32>;
    private i_vertices_h: StaticArray<f32>;
    private i_vertices_f: StaticArray<f32>;
    private i_vertices_close: StaticArray<bool>;
    private i_vertices_parent: StaticArray<i32>;

    //open list as a list for tasks in search algorithm
    private i_open_list: StaticArray<i32>;  // reserve the same size as vertices in the graph
    private i_open_length_length: i32;  // actual size of the task list

    //temp values for search process
    private t_min_f: f32 = 0.0;
    private t_min_vertex: i32 = 0;
    private t_min_vertex_index: i32 = 0;

    constructor(vertex_positions: StaticArray<f32> = new StaticArray<f32>(0), 
                vertices: StaticArray<i32> = new StaticArray<i32>(0),
                edges: StaticArray<i32> = new StaticArray<i32>(0)) {
        super();
        if(vertex_positions.length == 0 || vertices.length == 0 || edges.length == 0) {
            //if we create the graph with empty arrays, then it should be set empty
            //may be we load i from the binary data later
            this.m_positions = new StaticArray<f32>(0);
            this.m_vertex_names = new StaticArray<i32>(0);
            this.m_vertex_count = 0;
            this.m_edges = new StaticArray<i32>(0);
            this.m_index_map = new Map<i32, i32>();
            this.m_incident_map = new Map<i32, StaticArray<i32>>();
            this.i_vertices_g = new StaticArray<f32>(0);
            this.i_vertices_h = new StaticArray<f32>(0);
            this.i_vertices_f = new StaticArray<f32>(0);
            this.i_vertices_close = new StaticArray<bool>(0);
            this.i_vertices_parent = new StaticArray<i32>(0);
            this.i_open_list = new StaticArray<i32>(0);
            this.i_open_length_length = 0;
            this.t_min_f = 0.0;
            this.t_min_vertex = 0;
            this.t_min_vertex_index = 0;
        }
        else {
            this.m_positions = vertex_positions;
            this.m_vertex_names = vertices;
            this.m_edges = edges;

            this.m_vertex_count = vertices.length;

            //build index map
            let index_map = new Map<i32, i32>();
            for (let i = 0, len = vertices.length; i < len; i++) {
                index_map.set(vertices[i], i);
            }

            //build incident map
            //crate temp map with Array values
            let temp_map = new Map<i32, Array<i32>>();
            //init for all vertices
            for (let v = 0, len = vertices.length; v < len; v++) {
                temp_map.set(v, []);
            }
            //add vertex indices from edges
            for (let e = 0, len = edges.length / 2; e < len; e++) {
                let v1 = index_map.get(edges[2 * e + 0]);
                let v2 = index_map.get(edges[2 * e + 1]);

                temp_map.get(v1).push(v2);
                temp_map.get(v2).push(v1);
            }

            this.m_index_map = index_map;

            //copy to memory the map wih static array values
            let incident_map = new Map<i32, StaticArray<i32>>();
            let temp_keys = temp_map.keys();

            for (let i = 0, len = temp_keys.length; i < len; i++) {
                let v = temp_keys[i];
                let vs = temp_map.get(v);
                let vs_static = new StaticArray<i32>(vs.length);
                for (let j = 0, vs_len = vs.length; j < vs_len; j++) {
                    vs_static[j] = vs[j];
                }
                incident_map.set(v, vs_static);
            }

            this.m_incident_map = incident_map;

            //init inner arrays for search process
            let vertex_count = vertices.length;
            this.i_vertices_g = new StaticArray<f32>(vertex_count);
            this.i_vertices_h = new StaticArray<f32>(vertex_count);
            this.i_vertices_f = new StaticArray<f32>(vertex_count);
            this.i_vertices_close  = new StaticArray<bool>(vertex_count);
            let vertices_parent = new StaticArray<i32>(vertex_count);
            //set default values
            for (let i = 0; i < vertex_count; i++) {
                //default constructor for f32 arrays init it by 0.0, for bool array by false
                vertices_parent[i] = -1;
            }
            this.i_vertices_parent = vertices_parent;
            this.i_open_list = new StaticArray<i32>(vertex_count);
            this.i_open_length_length = 0;
        }
    }

    private _pre_start(target: i32): void {
        for (let i = 0, len = this.m_vertex_count; i < len; i++) {
            this.i_vertices_g[i] = 0.0;
            this.i_vertices_h[i] = this._get_distance_for_indexes(i, target);
            this.i_vertices_f[i] = 0.0;
            this.i_vertices_close[i] = false;
            this.i_vertices_parent[i] = -1;
        }
    }

    @inline
    private _get_distance_for_indexes(i: i32, j: i32): f32 {
        let positions = this.m_positions;
        return distance(
            positions[3 * i + 0],
            positions[3 * i + 1],
            positions[3 * i + 2],
            positions[3 * j + 0],
            positions[3 * j + 1],
            positions[3 * j + 2]
        );
    }

    private _edges_string(): string {
        let to_return = "";
        let edges = this.m_edges;
        for (let i = 0, len = edges.length / 2; i < len; i++) {
            to_return += "[" +
                edges[2 * i + 0].toString() + ", " +
                edges[2 * i + 1].toString() +
            "]";
            if (i < len - 1) {
                to_return += ", ";
            }
        }
        return to_return;
    }

    private _index_map_string(): string {
        let to_return = "{";
        let keys = this.m_index_map.keys();
        let values = this.m_index_map.values();
        for (let i = 0, len = keys.length; i < len; i++) {
            to_return += (
                keys[i].toString() + ": " +
                values[i].toString()
            );
            if (i < len - 1) {
                to_return += ", ";
            }
        }
        return to_return + "}";
    }

    private _positions_string(): string {
        let to_return = "";
        let positions = this.m_positions;
        for (let i = 0, len = this.m_vertex_count; i < len; i++) {
            to_return += "(" +
                positions[3 * i + 0].toString() + ", " +
                positions[3 * i + 1].toString() + ", " +
                positions[3 * i + 2].toString() +
            ")";
            if (i < len - 1) {
                to_return += ", ";
            }
        }
        return to_return;
    }

    private _incident_to_string(): string {
        let to_return = "{";
        let keys = this.m_incident_map.keys();
        for (let i = 0, len = keys.length; i < len; i++) {
            let key = keys[i];
            to_return += (
                key.toString() + ": " +
                this.m_incident_map.get(key).toString()
            );
            if (i < len - 1) {
                to_return += ", ";
            }
        }
        return to_return + "}";
    }

    search(start_vertex: i32, end_vertex: i32): StaticArray<i32> {
        //check are these vertices exists
        if (this.m_index_map.has(start_vertex) &&
            this.m_index_map.has(end_vertex)) {
            let start = this.m_index_map.get(start_vertex);
            let end = this.m_index_map.get(end_vertex);
            this._pre_start(end);

            //add first task to the open list
            this.i_open_list[0] = start;
            this.i_open_length_length = 1;
            // update values for the start vertex
            this.i_vertices_g[start] = 0.0;
            this.i_vertices_f[start] = this.i_vertices_h[start];

            //start iteration process
            while (this.i_open_length_length > 0) {
                let open_length_length = this.i_open_length_length;
                //search the minimum f-value for all vertices in the open list
                this.t_min_vertex = this.i_open_list[0];
                this.t_min_f = this.i_vertices_f[this.t_min_vertex];
                this.t_min_vertex_index = 0;

                //iterate by all values in the open list
                for (let i = 1; i < open_length_length; i++) {
                    let v = this.i_open_list[i];
                    let v_f = this.i_vertices_f[v];
                    if (v_f < this.t_min_f) {
                        this.t_min_f = v_f;
                        this.t_min_vertex = v;
                        this.t_min_vertex_index = i;
                    }
                }
                //remove from open list value at the index t_min_vertex_index
                this.i_open_list[this.t_min_vertex_index] = this.i_open_list[open_length_length - 1];
                this.i_open_length_length--;
                open_length_length--;
                //set selected vertex close
                this.i_vertices_close[this.t_min_vertex] = true;
                if (this.t_min_vertex == end) {
                    //we find the end vertex, so, build back path and return it

                    //TODO: use static buffer for constructing temp_path
                    let temp_path = new Array<i32>();  // this path will be reversed
                    while (this.i_vertices_parent[this.t_min_vertex] != -1) {
                        temp_path.push(this.t_min_vertex);
                        this.t_min_vertex = this.i_vertices_parent[this.t_min_vertex];
                    }
                    temp_path.push(this.t_min_vertex);
                    //form the streight path and apply their names
                    let temp_path_len = temp_path.length;
                    let to_return = new StaticArray<i32>(temp_path_len);
                    for (let i = 0; i < temp_path_len; i++) {
                        to_return[i] = this.m_vertex_names[temp_path[temp_path_len - 1 - i]];
                    }
                    return to_return;
                } else {
                    //add children of the selected vertex to the open list
                    let children = this.m_incident_map.get(this.t_min_vertex);
                    for (let i = 0; i < children.length; i++) {
                        let child_index = children[i];
                        if (!this.i_vertices_close[child_index]) {  // this child vertex is not close
                            let v_g = (
                                this._get_distance_for_indexes(this.t_min_vertex, child_index) +
                                    this.i_vertices_g[this.t_min_vertex]);
                            if (this.i_vertices_parent[child_index] == -1) {  // we come to the vertex at first time
                                this.i_vertices_parent[child_index] = this.t_min_vertex;
                                this.i_vertices_g[child_index] = v_g;
                                this.i_vertices_f[child_index] = v_g + this.i_vertices_h[child_index];
                                //add it to the open list
                                this.i_open_list[this.i_open_length_length] = child_index;
                                this.i_open_length_length++;
                            } else {  // this vertex already were visited

                                if (this.i_vertices_g[child_index] > v_g) {
                                    //redefine parent
                                    this.i_vertices_parent[child_index] = this.t_min_vertex;
                                    this.i_vertices_g[child_index] = v_g;
                                    this.i_vertices_f[child_index] = v_g + this.i_vertices_h[child_index];
                                }
                            }
                        }
                    }
                }
            }
        }

        return new StaticArray<i32>(0);
    }

    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_GRAPH);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;
        
        // positions
        let positions = staticarray_f32_to_bytes(this.m_positions);
        to_return.set(positions, shift);
        shift += staticarray_f32_bytes_length(this.m_positions);
        
        // vertex names
        let names = staticarray_i32_to_bytes(this.m_vertex_names);
        to_return.set(names, shift);
        shift += staticarray_i32_bytes_length(this.m_vertex_names)
        
        // vertex count
        let vertex_count = i32_to_bytes(this.m_vertex_count);
        to_return.set(vertex_count, shift);
        shift += i32_bytes_length();
        
        // edges
        let edges = staticarray_i32_to_bytes(this.m_edges);
        to_return.set(edges, shift);
        shift += staticarray_i32_bytes_length(this.m_edges)

        // index map
        let index_map = map_i32_i32_to_bytes(this.m_index_map);
        to_return.set(index_map, shift);
        shift += map_i32_i32_bytes_length(this.m_index_map);

        // incident map
        let incident_map = map_i32_staticarray_i32_to_bytes(this.m_incident_map);
        to_return.set(incident_map, shift);
        return to_return;
    }

    override from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start);
        const bytes_length = view.getInt32(start + 4);
        let shift = start;
        // shift is pointer to the start of the bytes section
        if(id == SD_TYPE.SD_TYPE_GRAPH) {
            shift += 8;
        } else { return; }

        // read positions
        const pos_id = view.getInt32(shift);
        if(pos_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const pos_bytes_length = view.getInt32(shift + 4);
            this.m_positions = staticarray_f32_from_bytes_expr(view, shift);
            shift += pos_bytes_length;
        } else { return; }

        // next read vertex names
        const names_id = view.getInt32(shift);
        const names_bytes_length = view.getInt32(shift + 4);
        if(names_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
            this.m_vertex_names = staticarray_i32_from_bytes_expr(view, shift);
            shift += names_bytes_length;
        } else { return; }

        // vertex count
        const count_id = view.getInt32(shift);
        const count_bytes_length = view.getInt32(shift + 4);
        if(count_id == SD_TYPE.SD_TYPE_INT32) {
            this.m_vertex_count = view.getInt32(shift + 8);
            shift += count_bytes_length;
        } else { return; }

        // edges
        const edges_id = view.getInt32(shift);
        const edges_bytes_length = view.getInt32(shift + 4);
        if(edges_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
            this.m_edges = staticarray_i32_from_bytes_expr(view, shift);
            shift += edges_bytes_length;
        } else { return; }

        // index map
        const index_id = view.getInt32(shift);
        const index_bytes_length = view.getInt32(shift + 4);
        if(index_id == SD_TYPE.SD_TYPE_MAP_INT32_INT32) {
            this.m_index_map = map_i32_i32_from_bytes_expr(view, shift);
            shift += index_bytes_length;
        } else { return; }

        // incident map
        const incident_id = view.getInt32(shift);
        const incident_bytes_length = view.getInt32(shift + 4);
        if(incident_id == SD_TYPE.SD_TYPE_MAP_INT32_STATICARRAY_INT32) {
            this.m_incident_map = map_i32_staticarray_i32_from_bytes_expr(view, shift);
            shift += incident_bytes_length;
        } else { return; }
        
        // finally recalculate all other class fields
        // this is the same as on constructor
        let vertex_count = this.m_vertex_count;
        this.i_vertices_g = new StaticArray<f32>(vertex_count);
        this.i_vertices_h = new StaticArray<f32>(vertex_count);
        this.i_vertices_f = new StaticArray<f32>(vertex_count);
        this.i_vertices_close  = new StaticArray<bool>(vertex_count);
        this.i_vertices_parent = new StaticArray<i32>(vertex_count);
        for (let i = 0; i < vertex_count; i++) {
            this.i_vertices_parent[i] = -1;
        }
        this.i_open_list = new StaticArray<i32>(vertex_count);
        this.i_open_length_length = 0;
    }

    override bytes_length(): u32 {
        return 4  // id
             + 4 // bytes length
             + staticarray_f32_bytes_length(this.m_positions)
             + staticarray_i32_bytes_length(this.m_vertex_names)
             + i32_bytes_length() // m_vertex_count
             + staticarray_i32_bytes_length(this.m_edges)
             + map_i32_i32_bytes_length(this.m_index_map)
             + map_i32_staticarray_i32_bytes_length(this.m_incident_map);
    }

    to_string(): string {
        return "<graph " + this.m_vertex_names.toString() +
               ", edges: " + this._edges_string() +
               ", map: " + this._index_map_string() +
               ", positions: " + this._positions_string() +
               ", incident: " + this._incident_to_string() +
               ">";
    }

    toString(): string {
        return this.to_string();
    }
}

export function staticarray_graph_bytes_length(graphs: StaticArray<Graph>): u32 {
    let to_return = 12;  // id, bytes length, count
    for(let i = 0, len = graphs.length; i < len; i++) {
        let graph = graphs[i];
        to_return += graph.bytes_length();
    }

    return to_return;
}

export function staticarray_graph_to_bytes(graphs: StaticArray<Graph>): Uint8Array {
    const bytes_length = staticarray_graph_bytes_length(graphs);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_STATICARRAY_GRAPH);
    view.setInt32(4, bytes_length);
    view.setInt32(8, graphs.length);
    let shift = 12;
    for(let i = 0, len = graphs.length; i < len; i++) {
        let graph = graphs[i];
        const graph_bytes_length = graph.bytes_length();
        to_return.set(graph.to_bytes(), shift);
        shift += graph_bytes_length;
    }
    return to_return;
}

export function staticarray_graph_from_bytes(bytes: Uint8Array): StaticArray<Graph> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        const id = view.getInt32(0);
        const bytes_length = view.getInt32(4);
        if(id == SD_TYPE.SD_TYPE_STATICARRAY_GRAPH) {
            const count = view.getInt32(8);
            let shift = 12;
            let to_return = new StaticArray<Graph>(count);
            for(let i = 0; i < count; i++) {
                const gr_bytes_length = view.getInt32(shift + 4);
                let graph = new Graph();
                graph.from_bytes(bytes.slice(shift, shift + gr_bytes_length));
                to_return[i] = graph;
                shift += gr_bytes_length;
            }

            return to_return;
        }
        else {
            return new StaticArray<Graph>(0);
        }
    }
    else {
        return new StaticArray<Graph>(0);
    }
}

export function staticarray_graph_from_bytes_expr(view: DataView, start: u32): StaticArray<Graph> {
    const count = view.getInt32(start + 8);
    let shift = start + 12;
    let to_return = new StaticArray<Graph>(count);
    for(let i = 0; i < count; i++) {
        const gr_bytes_length = view.getInt32(shift + 4);
        let graph = new Graph();
        graph.from_bytes(view, shift);
        to_return[i] = graph;
        shift += gr_bytes_length;
    }

    return to_return;
}
