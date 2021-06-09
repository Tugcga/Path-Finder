var STATIC_ARRAY_BUFFER_STEP: i32 = 8;  // when increase the data values count in static array and overflow it length, then we recreate new bigger array

export class NavmeshNode{
    m_vertices: StaticArray<f32>;
    m_polygon: StaticArray<i32>;
    m_length: i32;  // the number of corners in the polygon
    m_index: i32;  // index = -1 mean, that node is empty
    m_group: i32;
    m_neighbor: StaticArray<i32>;  // manualy increase the buffer, values are indexes of neighborhood polygons
    m_neighbor_count: i32;
    m_center: StaticArray<f32>;
    m_normal: StaticArray<f32>;
    m_vertex_normals: StaticArray<f32>;  // packed per-vertex [x, y, z for the first vertex, then for the second, ...]
    m_portals: Map<i32, StaticArray<f32>>;  // key - node index, value - array of length 6 with vertex coordinates

    constructor(all_vertices: StaticArray<f32>, index: i32, polygon_indexes: StaticArray<i32>) {
        this.m_length = polygon_indexes.length;
        this.m_vertices = new StaticArray<f32>(3 * this.m_length);
        this.m_index = index;
        for(let i: i32 = 0; i < this.m_length; i++){
            let k: i32 = polygon_indexes[i];
            for(let j: i32 = 0; j < 3; j++){
                this.m_vertices[3*i + j] = all_vertices[3*k + j]
            }
        }

        this.m_polygon = polygon_indexes;
        this.m_group = -1;
        this.m_neighbor = new StaticArray<i32>(0);
        this.m_neighbor_count = 0;

        //calculate polygon center
        this.m_center = new StaticArray<f32>(3);
        this.m_normal = new StaticArray<f32>(3);
        this.m_vertex_normals = new StaticArray<f32>(3 * this.m_length);
        this.m_portals = new Map<i32, StaticArray<f32>>();
        //init zero values
        for(let i: i32 = 0; i < 3; i++){
            this.m_center[i] = 0.0;
            this.m_normal[i] = 0.0;
        }
        for(let i: i32 = 0; i < this.m_length; i++){
            for(let j: i32 = 0; j < 3; j++){
                this.m_center[j] += this.m_vertices[3*i + j]
            }
        }
        if(this.m_length > 0){
            for(let i: i32 = 0; i < 3; i++){
                this.m_center[i] = this.m_center[i] / <f32>this.m_length;
            }
        }
        //after center we can calculate average normal of the polygon
        for(let i: i32 = 0; i < this.m_length; i++){
            let j: i32 = i + 1;
            if(j == this.m_length){
                j = 0;
            }
            let c_x: f32 = this._cross_x(this.m_vertices[3*i + 1] - this.m_center[1], this.m_vertices[3*i + 2] - this.m_center[2], this.m_vertices[3*j + 1] - this.m_center[1], this.m_vertices[3*j + 2] - this.m_center[2]);
            let c_y: f32 = this._cross_y(this.m_vertices[3*i] - this.m_center[0], this.m_vertices[3*i + 2] - this.m_center[2], this.m_vertices[3*j] - this.m_center[0], this.m_vertices[3*j + 2] - this.m_center[2]);
            let c_z: f32 = this._cross_z(this.m_vertices[3*i] - this.m_center[0], this.m_vertices[3*i + 1] - this.m_center[1], this.m_vertices[3*j] - this.m_center[0], this.m_vertices[3*j + 1] - this.m_center[1]);
            this.m_normal[0] += c_x;
            this.m_normal[1] += c_y;
            this.m_normal[2] += c_z;
        }
        //normalize the normal
        if(this.m_length > 0){
            let normal_length: f32 = <f32>Math.sqrt(this.m_normal[0]*this.m_normal[0] + this.m_normal[1]*this.m_normal[1] + this.m_normal[2]*this.m_normal[2]);
            this.m_normal[0] = this.m_normal[0] / normal_length;
            this.m_normal[1] = this.m_normal[1] / normal_length;
            this.m_normal[2] = this.m_normal[2] / normal_length;
        }

        //vertex normals
        for(let i: i32 = 0; i < 3 * this.m_length; i++){
            this.m_vertex_normals[i] = 0.0;
        }
        for(let i: i32 = 0; i < this.m_length; i++){
            let j: i32 = i + 1;
            if(j == this.m_length){
                j = 0;
            }
            let k: i32 = j + 1;
            if(k == this.m_length){
                k = 0;
            }
            let v_x: f32 = this._cross_x(this.m_vertices[3*j + 1] - this.m_vertices[3*i + 1], this.m_vertices[3*j + 2] - this.m_vertices[3*i + 2], this.m_vertices[3*k + 1] - this.m_vertices[3*i + 1], this.m_vertices[3*k + 2] - this.m_vertices[3*i + 2]);
            let v_y: f32 = this._cross_y(this.m_vertices[3*j] - this.m_vertices[3*i], this.m_vertices[3*j + 2] - this.m_vertices[3*i + 2], this.m_vertices[3*k] - this.m_vertices[3*i], this.m_vertices[3*k + 2] - this.m_vertices[3*i + 2]);
            let v_z: f32 = this._cross_z(this.m_vertices[3*j] - this.m_vertices[3*i], this.m_vertices[3*j + 1] - this.m_vertices[3*i + 1], this.m_vertices[3*k] - this.m_vertices[3*i], this.m_vertices[3*k + 1] - this.m_vertices[3*i + 1]);
            let d: f32 = <f32>Math.sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
            this.m_vertex_normals[3*i] = v_x / d;
            this.m_vertex_normals[3*i + 1] = v_y / d;
            this.m_vertex_normals[3*i + 2] = v_z / d;
        }
    }

    add_neighbor(node_index: i32, v0_x: f32, v0_y: f32, v0_z: f32, v1_x: f32, v1_y: f32, v1_z: f32): void{
        if(!this.m_portals.has(node_index)){
            var new_portal: StaticArray<f32> = new StaticArray<f32>(6);
            new_portal[0] = v0_x; new_portal[1] = v0_y; new_portal[2] = v0_z;
            new_portal[3] = v1_x; new_portal[4] = v1_y; new_portal[5] = v1_z;
            this.m_portals.set(node_index, new_portal);

            if(this.m_neighbor_count == this.m_neighbor.length){
                //create new neighbor buffer
                var new_neighbor: StaticArray<i32> = new StaticArray<i32>(this.m_neighbor.length + STATIC_ARRAY_BUFFER_STEP);
                //copy values to them
                for(let i: i32 = 0; i < this.m_neighbor.length; i++){
                    new_neighbor[i] = this.m_neighbor[i];
                }
                //rewrite neigboor link
                this.m_neighbor = new_neighbor;
            }

            this.m_neighbor_count++;
            this.m_neighbor[this.m_neighbor_count - 1] = node_index;
        }
    }

    get_portal(node_index: i32): StaticArray<f32>{
        if(this.m_portals.has(node_index)){
            return this.m_portals.get(node_index);
        }
        else{
            return new StaticArray<f32>(0);
        }
    }

    get_neighbord(): StaticArray<i32>{
        //for return we create new array of actual size
        //in main program this method called only on constructor, so, it does not use GC many times
        var to_return: StaticArray<i32> = new StaticArray<i32>(this.m_neighbor_count);
        for(let i: i32 = 0; i < this.m_neighbor_count; i++){
            to_return[i] = this.m_neighbor[i];
        }
        return to_return;
    }

    get_index(): i32{
        return this.m_index;
    }

    get_group(): i32{
        return this.m_group;
    }

    get_vertex_indexes(): StaticArray<i32>{
        return this.m_polygon;
    }

    get_vertex_coordinates(): StaticArray<f32>{
        return this.m_vertices;
    }

    get_normal(): StaticArray<f32>{
        return this.m_normal;
    }

    get_center(): StaticArray<f32>{
        return this.m_center;
    }

    set_group(group_index: i32, group_array: Array<i32>, all_nodes: StaticArray<NavmeshNode>): void{
        if(this.m_group == -1){
            this.m_group = group_index;
            group_array.push(this.m_index);  // <-- recreate array here
            for(let i: i32 = 0; i < this.m_neighbor_count; i++){
                all_nodes[this.m_neighbor[i]].set_group(group_index, group_array, all_nodes);
            }
        }
    }

    is_point_inside(p_x: f32, p_y: f32, p_z: f32): boolean{
        for(let i: i32 = 0; i < this.m_length; i++){
            let j: i32 = i + 1;
            if(j == this.m_length){
                j = 0;
            }
            let v_x: f32 = this._cross_x(this.m_vertices[3*j + 1] - this.m_vertices[3*i + 1], this.m_vertices[3*j + 2] - this.m_vertices[3*i + 2], p_y - this.m_vertices[3*i + 1], p_z - this.m_vertices[3*i + 2]);
            let v_y: f32 = this._cross_y(this.m_vertices[3*j] - this.m_vertices[3*i], this.m_vertices[3*j + 2] - this.m_vertices[3*i + 2], p_x - this.m_vertices[3*i], p_z - this.m_vertices[3*i + 2]);
            let v_z: f32 = this._cross_z(this.m_vertices[3*j] - this.m_vertices[3*i], this.m_vertices[3*j + 1] - this.m_vertices[3*i + 1], p_x - this.m_vertices[3*i], p_y - this.m_vertices[3*i + 1]);
            let d: f32 = v_x * this.m_vertex_normals[3*i] + v_y * this.m_vertex_normals[3*i + 1] + v_z * this.m_vertex_normals[3*i + 2];
            if(d < 0.0){
                return false;
            }
        }
        return true;
    }

    _cross_x(a_y: f32, a_z: f32, b_y: f32, b_z: f32): f32{
        return a_y*b_z - a_z*b_y;
    }

    _cross_y(a_x: f32, a_z: f32, b_x: f32, b_z: f32): f32{
        return a_z*b_x - a_x*b_z;
    }

    _cross_z(a_x: f32, a_y: f32, b_x: f32, b_y: f32): f32{
        return a_x*b_y - a_y*b_x;
    }

    _cross(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): StaticArray<f32>{
        var to_return: StaticArray<f32> = new StaticArray<f32>(3);
        to_return[0] = a_y*b_z - a_z*b_y;
        to_return[1] = a_z*b_x - a_x*b_z;
        to_return[2] = a_x*b_y - a_y*b_x;
        return to_return;
    }

    _portals_to_string(): string{
        var to_return: string = "{";
        let keys: Array<i32> = this.m_portals.keys();
        for(let i: i32 = 0; i < keys.length; i++){
            to_return += keys[i].toString() + ": " + this.m_portals.get(keys[i]).toString();
            if(i < keys.length - 1){
                to_return += ", ";
            }
        }
        to_return += "}";
        return to_return;
    }

    to_string(): string{
        return "<node " + this.m_index.toString() + 
               ", vertices: " + this.m_polygon.toString() + 
               ", positions: " + this.m_vertices.toString() +
               ", center: " + this.m_center.toString() +
               ", normal: " + this.m_normal.toString() +
               ", vertex normals: " + this.m_vertex_normals.toString() +
               ", group: " + this.m_group.toString() +
               ", neighbor: " + this.m_neighbor.toString() + " (" + this.m_neighbor_count.toString() + ")" +
               ", portals: " + this._portals_to_string() +
               ">";
    }
}