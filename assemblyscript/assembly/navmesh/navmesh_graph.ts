import { distance } from "../common/utilities";

export class Graph {
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
    private t_min_f: f32;
    private t_min_vertex: i32;
    private t_min_vertex_index: i32;

    constructor(vertex_positions: StaticArray<f32>, vertices: StaticArray<i32>, edges: StaticArray<i32>) {
        this.m_positions = vertex_positions;
        this.m_vertex_names = vertices;
        this.m_edges = edges;

        this.m_vertex_count = vertices.length;

        //build index map
        this.m_index_map = new Map<i32, i32>();
        for (let i = 0, len = this.m_vertex_count; i < len; i++) {
            this.m_index_map.set(unchecked(this.m_vertex_names[i]), i);
        }

        //build incident map
        //crate temp map with Array values
        let temp_map = new Map<i32, Array<i32>>();
        //init for all vertices
        for (let v = 0, len = this.m_vertex_count; v < len; v++) {
            temp_map.set(v, []);
        }
        //add vertex indices from edges
        for (let e = 0, len = this.m_edges.length / 2; e < len; e++) {
            let v1 = this.m_index_map.get(unchecked(this.m_edges[2 * e + 0]));
            let v2 = this.m_index_map.get(unchecked(this.m_edges[2 * e + 1]));

            temp_map.get(v1).push(v2);
            temp_map.get(v2).push(v1);
        }

        //copy to memory the map wih static array values
        this.m_incident_map = new Map<i32, StaticArray<i32>>();
        let temp_keys = temp_map.keys();

        for (let i = 0, len = temp_keys.length; i < len; i++) {
            let v = unchecked(temp_keys[i]);
            let vs = temp_map.get(v);
            let vs_static = new StaticArray<i32>(vs.length);
            for (let j = 0, vs_len = vs.length; j < vs_len; j++) {
                unchecked(vs_static[j] = vs[j]);
            }
            this.m_incident_map.set(v, vs_static);
        }

        //init inner arrays for search process
        let vertex_count = this.m_vertex_count;
        this.i_vertices_g = new StaticArray<f32>(vertex_count);
        this.i_vertices_h = new StaticArray<f32>(vertex_count);
        this.i_vertices_f = new StaticArray<f32>(vertex_count);
        this.i_vertices_close  = new StaticArray<bool>(vertex_count);
        this.i_vertices_parent = new StaticArray<i32>(vertex_count);
        //set default values
        for (let i = 0; i < vertex_count; i++) {
            //default constructor for f32 arrays init it by 0.0, for bool array by false
            unchecked(this.i_vertices_parent[i] = -1);
        }
        this.i_open_list = new StaticArray<i32>(vertex_count);
        this.i_open_length_length = 0;
    }

    private _pre_start(target: i32): void {
        for (let i = 0, len = this.m_vertex_count; i < len; i++) {
            unchecked(this.i_vertices_g[i] = 0.0);
            unchecked(this.i_vertices_h[i] = this._get_distance_for_indexes(i, target));
            unchecked(this.i_vertices_f[i] = 0.0);
            unchecked(this.i_vertices_close[i] = false);
            unchecked(this.i_vertices_parent[i] = -1);
        }
    }

    @inline
    private _get_distance_for_indexes(i: i32, j: i32): f32 {
        let positions = this.m_positions;
        return distance(
            unchecked(positions[3 * i + 0]),
            unchecked(positions[3 * i + 1]),
            unchecked(positions[3 * i + 2]),
            unchecked(positions[3 * j + 0]),
            unchecked(positions[3 * j + 1]),
            unchecked(positions[3 * j + 2])
        );
    }

    private _edges_string(): string {
        let to_return = "";
        let edges = this.m_edges;
        for (let i = 0, len = edges.length / 2; i < len; i++) {
            to_return += "[" +
                unchecked(edges[2 * i + 0]).toString() + ", " +
                unchecked(edges[2 * i + 1]).toString() +
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
                unchecked(keys[i]).toString() + ": " +
                unchecked(values[i]).toString()
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
                unchecked(positions[3 * i + 0]).toString() + ", " +
                unchecked(positions[3 * i + 1]).toString() + ", " +
                unchecked(positions[3 * i + 2]).toString() +
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
            let key = unchecked(keys[i]);
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
            unchecked(this.i_open_list[0] = start);
            this.i_open_length_length = 1;
            // update values for the start vertex
            unchecked(this.i_vertices_g[start] = 0.0);
            unchecked(this.i_vertices_f[start] = this.i_vertices_h[start]);

            //start iteration process
            while (this.i_open_length_length > 0) {
                let open_length_length = this.i_open_length_length;
                //search the minimum f-value for all vertices in the open list
                this.t_min_vertex = unchecked(this.i_open_list[0]);
                this.t_min_f = unchecked(this.i_vertices_f[this.t_min_vertex]);
                this.t_min_vertex_index = 0;

                //iterate by all values in the open list
                for (let i = 1; i < open_length_length; i++) {
                    let v = unchecked(this.i_open_list[i]);
                    let v_f = unchecked(this.i_vertices_f[v]);
                    if (v_f < this.t_min_f) {
                        this.t_min_f = v_f;
                        this.t_min_vertex = v;
                        this.t_min_vertex_index = i;
                    }
                }
                //remove from open list value at the index t_min_vertex_index
                unchecked(this.i_open_list[this.t_min_vertex_index] = this.i_open_list[open_length_length - 1]);
                this.i_open_length_length--;
                open_length_length--;
                //set selected vertex close
                unchecked(this.i_vertices_close[this.t_min_vertex] = true);
                if (this.t_min_vertex == end) {
                    //we find the end vertex, so, build back path and return it

                    //TODO: use static buffer for constructing temp_path
                    let temp_path = new Array<i32>();  // this path will be reversed
                    while (unchecked(this.i_vertices_parent[this.t_min_vertex]) != -1) {
                        temp_path.push(this.t_min_vertex);
                        this.t_min_vertex = unchecked(this.i_vertices_parent[this.t_min_vertex]);
                    }
                    temp_path.push(this.t_min_vertex);
                    //form the streight path and apply their names
                    let temp_path_len = temp_path.length;
                    let to_return = new StaticArray<i32>(temp_path_len);
                    for (let i = 0; i < temp_path_len; i++) {
                        unchecked(to_return[i] = this.m_vertex_names[temp_path[temp_path_len - 1 - i]]);
                    }
                    return to_return;
                } else {
                    //add children of the selected vertex to the open list
                    let children = this.m_incident_map.get(this.t_min_vertex);
                    for (let i = 0; i < children.length; i++) {
                        let child_index = unchecked(children[i]);
                        if (!unchecked(this.i_vertices_close[child_index])) {  // this child vertex is not close
                            let v_g = (
                                this._get_distance_for_indexes(this.t_min_vertex, child_index) +
                                unchecked(this.i_vertices_g[this.t_min_vertex])
                            );
                            if (unchecked(this.i_vertices_parent[child_index]) == -1) {  // we come to the vertex at first time
                                unchecked(this.i_vertices_parent[child_index] = this.t_min_vertex);
                                unchecked(this.i_vertices_g[child_index] = v_g);
                                unchecked(this.i_vertices_f[child_index] = v_g + this.i_vertices_h[child_index]);
                                //add it to the open list
                                unchecked(this.i_open_list[this.i_open_length_length] = child_index);
                                this.i_open_length_length++;
                            } else {  // this vertex already were visited

                                if (unchecked(this.i_vertices_g[child_index]) > v_g) {
                                    //redefine parent
                                    unchecked(this.i_vertices_parent[child_index] = this.t_min_vertex);
                                    unchecked(this.i_vertices_g[child_index] = v_g);
                                    unchecked(this.i_vertices_f[child_index] = v_g + this.i_vertices_h[child_index]);
                                }
                            }
                        }
                    }
                }
            }
        }

        return new StaticArray<i32>(0);
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
