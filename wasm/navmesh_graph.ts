export class INavmeshGraph{
    m_positions: StaticArray<f32>;  // store by triples [x1, y1, z1, x2, y2, z2, ...]
    m_vertex_names: StaticArray<i32>;  // in the same order as positions
    m_vertex_count: i32;
    m_edges: StaticArray<i32>;  // store by pairs [s1, e1, s2, e2, ...]
    m_index_map: Map<i32, i32>;
    m_incident_map: Map<i32, StaticArray<i32>>; // key - vertex, value - incident vertices (not names, but indices from 0 to n-1)

    //inner arrays for search algorithm
    i_vertices_g: StaticArray<f32>;
    i_vertices_h: StaticArray<f32>;
    i_vertices_f: StaticArray<f32>;
    i_vertices_close: StaticArray<boolean>;
    i_vertices_parent: StaticArray<i32>;

    //open list as a list for tasks in search algorithm
    i_open_list: StaticArray<i32>;  // reserve the same size as vertices in the graph
    i_open_length_length: i32;  // actual size of the task list

    //temp values for search process
    t_min_f: f32;
    t_min_vertex: i32;
    t_min_vertex_index: i32;

    constructor(vertex_positions: StaticArray<f32>, vertices: StaticArray<i32>, edges: StaticArray<i32>) {
        this.m_positions = vertex_positions;
        this.m_vertex_names = vertices;
        this.m_edges = edges;
        
        this.m_vertex_count = vertices.length;

        //build index map
        this.m_index_map = new Map<i32, i32>();
        for(let i: i32 = 0; i < this.m_vertex_count; i++){
            this.m_index_map.set(this.m_vertex_names[i], i);
        }

        //build incident map
        //crate temp map with Array values
        let temp_map: Map<i32, Array<i32>> = new Map<i32, Array<i32>>();
        //init for all vertices
        for(let v: i32 = 0; v < this.m_vertex_count; v++){
            temp_map.set(v, new Array<i32>());
        }
        //add vertex indices from edges
        for(let e: i32 = 0; e < this.m_edges.length / 2; e++){
            let v1: i32 = this.m_index_map.get(this.m_edges[2*e]);
            let v2: i32 = this.m_index_map.get(this.m_edges[2*e + 1]);

            temp_map.get(v1).push(v2);
            temp_map.get(v2).push(v1);
        }

        //copy to memory map wih static array values
        this.m_incident_map = new Map<i32, StaticArray<i32>>();
        let temp_keys: Array<i32> = temp_map.keys();
        for(let i: i32 = 0; i < temp_keys.length; i++){
            let v: i32 = temp_keys[i];
            let vs: Array<i32> = temp_map.get(v);
            let vs_static: StaticArray<i32> = new StaticArray<i32>(vs.length);
            for(let j: i32 = 0; j < vs.length; j++){
                vs_static[j] = vs[j];
            }
            this.m_incident_map.set(v, vs_static);
        }

        //init inner arrays for search process
        this.i_vertices_g = new StaticArray<f32>(this.m_vertex_count);
        this.i_vertices_h = new StaticArray<f32>(this.m_vertex_count);
        this.i_vertices_f = new StaticArray<f32>(this.m_vertex_count);
        this.i_vertices_close = new StaticArray<boolean>(this.m_vertex_count);
        this.i_vertices_parent = new StaticArray<i32>(this.m_vertex_count);
        //set default values
        for(let i: i32 = 0; i < this.m_vertex_count; i++){
            this.i_vertices_g[i] = 0.0;
            this.i_vertices_h[i] = 0.0;
            this.i_vertices_f[i] = 0.0;
            this.i_vertices_close[i] = false;
            this.i_vertices_parent[i] = -1;
        }
        this.i_open_list = new StaticArray<i32>(this.m_vertex_count);
        this.i_open_length_length = 0;
    }

    _pre_start(target: i32): void{
        for(let i: i32 = 0; i < this.m_vertex_count; i++){
            this.i_vertices_g[i] = 0.0;
            this.i_vertices_h[i] = this._get_distance_for_indexes(i, target);
            this.i_vertices_f[i] = 0.0;
            this.i_vertices_close[i] = false;
            this.i_vertices_parent[i] = -1;
        }
    }

    _get_distance(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): f32{
        return <f32>Math.sqrt((a_x - b_x)*(a_x - b_x) + (a_y - b_y)*(a_y - b_y) + (a_z - b_z)*(a_z - b_z));
    }

    _get_distance_for_indexes(i: i32, j: i32): f32{
        return this._get_distance(this.m_positions[3*i], this.m_positions[3*i + 1], this.m_positions[3*i + 2],
                                  this.m_positions[3*j], this.m_positions[3*j + 1], this.m_positions[3*j + 2]);
    }

    _edges_string(): string{
        let to_return: string = "";
        let edge_count: i32 = this.m_edges.length / 2;
        for(let i: i32 = 0; i < edge_count; i++){
            to_return += "[" + this.m_edges[2*i].toString() + ", " + this.m_edges[2*i+1].toString() + "]";
            if(i < edge_count - 1){
                to_return += ", ";
            }
        }
        return to_return;
    }

    _index_map_string(): string{
        let to_return: string = "{";
        let keys: Array<i32> = this.m_index_map.keys();
        let values: Array<i32> = this.m_index_map.values();
        for(let i: i32 = 0; i < keys.length; i++){
            to_return += keys[i].toString() + ": " + values[i].toString();
            if(i < keys.length - 1){
                to_return += ", ";
            }
        }
        to_return += "}";
        return to_return;
    }

    _positions_string(): string{
        let to_return: string = "";
        for(let i: i32 = 0; i < this.m_vertex_count; i++){
            to_return += "(" + this.m_positions[3*i].toString() + ", " + this.m_positions[3*i+1].toString() + ", " + this.m_positions[3*i+2].toString() + ")";
            if(i < this.m_vertex_count - 1){
                to_return += ", ";
            }
        }
        return to_return;
    }

    _incident_to_string(): string{
        let to_return = "{";
        let keys: Array<i32> = this.m_incident_map.keys();
        for(let i: i32 = 0; i < keys.length; i++){
            to_return += keys[i].toString() + ": " + this.m_incident_map.get(keys[i]).toString();
            if(i < keys.length - 1){
                to_return += ", ";
            }
        }
        to_return += "}";
        return to_return;
    }

    search(start_vertex: i32, end_vertex: i32): Int32Array{
        //check are these verticeas exists
        if(this.m_index_map.has(start_vertex) && this.m_index_map.has(end_vertex)){
            let start: i32 = this.m_index_map.get(start_vertex);
            let end: i32 = this.m_index_map.get(end_vertex);
            this._pre_start(end);

            //add first task to the open list
            this.i_open_list[0] = start;
            this.i_open_length_length = 1;
            // update values for the start vertex
            this.i_vertices_g[start] = 0.0;
            this.i_vertices_f[start] = this.i_vertices_h[start];

            //start iteration process
            while(this.i_open_length_length > 0){
                //search the minimum f-value for all vertices in the open list
                this.t_min_vertex = this.i_open_list[0]
                this.t_min_f = this.i_vertices_f[this.t_min_vertex];
                this.t_min_vertex_index = 0;

                //iterate by all values in the open list
                for(let i: i32 = 1; i < this.i_open_length_length; i++){
                    let v: i32 = this.i_open_list[i];
                    let v_f: f32 = this.i_vertices_f[v];
                    if(v_f < this.t_min_f){
                        this.t_min_f = v_f;
                        this.t_min_vertex = v;
                        this.t_min_vertex_index = i;
                    }
                }
                //remove from open list value at the index t_min_vertex_index
                this.i_open_list[this.t_min_vertex_index] = this.i_open_list[this.i_open_length_length - 1];
                this.i_open_length_length--;
                //set selected vertex close
                this.i_vertices_close[this.t_min_vertex] = true;
                if(this.t_min_vertex == end){
                    //we find the end vertex, so, build back path and return it

                    //TODO: use static buffer for constructing temp_path
                    let temp_path: Array<i32> = new Array<i32>();  // this path will be reversed
                    while(this.i_vertices_parent[this.t_min_vertex] != -1){
                        temp_path.push(this.t_min_vertex);
                        this.t_min_vertex = this.i_vertices_parent[this.t_min_vertex];
                    }
                    temp_path.push(this.t_min_vertex);
                    //form the streight path and apply their names
                    let to_return: Int32Array = new Int32Array(temp_path.length);
                    for(let i: i32 = 0; i < temp_path.length; i++){
                        to_return[i] = this.m_vertex_names[temp_path[temp_path.length - 1 - i]];
                    }
                    return to_return;
                }
                else{
                    //add children of the selected vertex to the open list
                    let children: StaticArray<i32> = this.m_incident_map.get(this.t_min_vertex);
                    for(let i: i32 = 0; i < children.length; i++){
                        let child_index: i32 = children[i];
                        if(!this.i_vertices_close[child_index]){  // this child vertex is not close
                            let v_g: f32 = this._get_distance_for_indexes(this.t_min_vertex, child_index) + this.i_vertices_g[this.t_min_vertex];
                            if(this.i_vertices_parent[child_index] == -1){  // we come to the vertex at first time
                                this.i_vertices_parent[child_index] = this.t_min_vertex;
                                this.i_vertices_g[child_index] = v_g;
                                this.i_vertices_f[child_index] = v_g + this.i_vertices_h[child_index];
                                //add it to the open list
                                this.i_open_list[this.i_open_length_length] = child_index;
                                this.i_open_length_length++;
                            }
                            else{  // this vertex already were visited
                                if(this.i_vertices_g[child_index] > v_g){
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

        return new Int32Array(0);
    }

    to_string(): string{
        return "<graph " + this.m_vertex_names.toString() + 
               ", edges: " + this._edges_string() + 
               ", map: " + this._index_map_string() + 
               ", positions: " + this._positions_string() + 
               ", incident: " + this._incident_to_string() +
               ">";
    }

    toString(): string{
        return this.to_string();
    }
}