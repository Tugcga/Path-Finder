var STATIC_ARRAY_BUFFER_STEP: i32 = 8;  // when increase the data values count in static array and overflow it length, then we recreate new bigger array

export class NavmeshNode {
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
        let indexes_len = polygon_indexes.length;

        this.m_length = indexes_len;
        this.m_index = index;

        let vertices = new StaticArray<f32>(3 * indexes_len);
        this.m_vertices = vertices;

        for (let i = 0; i < indexes_len; i++) {
            let k = unchecked(polygon_indexes[i]);
            for (let j = 0; j < 3; j++) {
                unchecked(vertices[3 * i + j] = all_vertices[3 * k + j]);
            }
        }

        this.m_polygon = polygon_indexes;
        this.m_group = -1;
        this.m_neighbor = new StaticArray<i32>(0);
        this.m_neighbor_count = 0;

        //calculate polygon center
        this.m_center = new StaticArray<f32>(3);
        this.m_normal = new StaticArray<f32>(3);
        this.m_vertex_normals = new StaticArray<f32>(3 * indexes_len);
        this.m_portals = new Map<i32, StaticArray<f32>>();
        // init zero values. Unnecessary in AS
        // for (let i = 0; i < 3; i++){
        //     this.m_center[i] = 0.0;
        //     this.m_normal[i] = 0.0;
        // }

        let cx = unchecked(this.m_center[0]);
        let cy = unchecked(this.m_center[1]);
        let cz = unchecked(this.m_center[2]);

        for (let i = 0; i < indexes_len; i++) {
            cx += unchecked(vertices[3 * i + 0]);
            cy += unchecked(vertices[3 * i + 1]);
            cz += unchecked(vertices[3 * i + 2]);
        }
        if (indexes_len > 0) {
            let invLen: f32 = 1.0 / <f32>indexes_len;
            cx *= invLen;
            cy *= invLen;
            cz *= invLen;
        }

        unchecked(this.m_center[0] = cx);
        unchecked(this.m_center[1] = cy);
        unchecked(this.m_center[2] = cz);

        //after center we can calculate average normal of the polygon
        for (let i = 0; i < indexes_len; i++) {
            let j = i + 1;
            if (j == indexes_len) {
                j = 0;
            }
            let c_x = this._cross_x(
                unchecked(vertices[3 * i + 1]) - cy,
                unchecked(vertices[3 * i + 2]) - cz,
                unchecked(vertices[3 * j + 1]) - cy,
                unchecked(vertices[3 * j + 2]) - cz
            );
            let c_y = this._cross_y(
                unchecked(vertices[3 * i + 0]) - cx,
                unchecked(vertices[3 * i + 2]) - cz,
                unchecked(vertices[3 * j + 0]) - cx,
                unchecked(vertices[3 * j + 2]) - cz
            );
            let c_z = this._cross_z(
                unchecked(vertices[3 * i + 0]) - cx,
                unchecked(vertices[3 * i + 1]) - cy,
                unchecked(vertices[3 * j + 0]) - cx,
                unchecked(vertices[3 * j + 1]) - cy
            );
            unchecked(this.m_normal[0] += c_x);
            unchecked(this.m_normal[1] += c_y);
            unchecked(this.m_normal[2] += c_z);
        }
        //normalize the normal
        if (indexes_len > 0) {
            let nx = unchecked(this.m_normal[0]);
            let ny = unchecked(this.m_normal[1]);
            let nz = unchecked(this.m_normal[2]);
            let invLen: f32 = 1.0 / Mathf.sqrt(nx * nx + ny * ny + nz * nz);
            unchecked(this.m_normal[0] = nx * invLen);
            unchecked(this.m_normal[1] = ny * invLen);
            unchecked(this.m_normal[2] = nz * invLen);
        }

        let vertex_normals = this.m_vertex_normals;

        //vertex normals
        for (let i = 0, len = 3 * indexes_len; i < len; i++) {
            unchecked(vertex_normals[i] = 0.0);
        }

        for (let i = 0; i < indexes_len; i++) {
            let j = i + 1;
            if (j == indexes_len) {
                j = 0;
            }
            let k = j + 1;
            if (k == indexes_len) {
                k = 0;
            }
            let v_x = this._cross_x(
                unchecked(vertices[3 * j + 1] - vertices[3 * i + 1]),
                unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                unchecked(vertices[3 * k + 1] - vertices[3 * i + 1]),
                unchecked(vertices[3 * k + 2] - vertices[3 * i + 2])
            );
            let v_y = this._cross_y(
                unchecked(vertices[3 * j + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                unchecked(vertices[3 * k + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * k + 2] - vertices[3 * i + 2])
            );
            let v_z = this._cross_z(
                unchecked(vertices[3 * j + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * j + 1] - vertices[3 * i + 1]),
                unchecked(vertices[3 * k + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * k + 1] - vertices[3 * i + 1])
            );

            let inv_d: f32 = 1.0 / Mathf.sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
            unchecked(vertex_normals[3 * i + 0] = v_x * inv_d);
            unchecked(vertex_normals[3 * i + 1] = v_y * inv_d);
            unchecked(vertex_normals[3 * i + 2] = v_z * inv_d);
        }
    }

    add_neighbor(node_index: i32, v0_x: f32, v0_y: f32, v0_z: f32, v1_x: f32, v1_y: f32, v1_z: f32): void {
        if (!this.m_portals.has(node_index)) {
            var new_portal = new StaticArray<f32>(6);
            unchecked(new_portal[0] = v0_x);
            unchecked(new_portal[1] = v0_y);
            unchecked(new_portal[2] = v0_z);
            unchecked(new_portal[3] = v1_x);
            unchecked(new_portal[4] = v1_y);
            unchecked(new_portal[5] = v1_z);

            this.m_portals.set(node_index, new_portal);

            let neighbor = this.m_neighbor;
            let neighbor_len = this.m_neighbor.length;
            if (this.m_neighbor_count == neighbor_len) {
                //create new neighbor buffer
                var new_neighbor = new StaticArray<i32>(neighbor_len + STATIC_ARRAY_BUFFER_STEP);
                //copy values to them
                for (let i = 0; i < neighbor_len; i++) {
                    unchecked(new_neighbor[i] = neighbor[i]);
                }
                //rewrite neigboor link
                this.m_neighbor = new_neighbor;
            }

            this.m_neighbor_count++;
            unchecked(neighbor[this.m_neighbor_count - 1] = node_index);
        }
    }

    @inline
    get_portal(node_index: i32): StaticArray<f32> {
        let portals = this.m_portals;
        if (portals.has(node_index)) {
            return portals.get(node_index);
        } else {
            return new StaticArray<f32>(0);
        }
    }

    get_neighbord(): StaticArray<i32> {
        //for return we create new array of actual size
        //in main program this method called only on constructor, so, it does not use GC many times
        let neighbor = this.m_neighbor;
        let neighbor_count = this.m_neighbor_count;
        let to_return = new StaticArray<i32>(neighbor_count);
        for (let i = 0; i < neighbor_count; i++) {
            unchecked(to_return[i] = neighbor[i]);
        }
        return to_return;
    }

    @inline
    get_index(): i32 {
        return this.m_index;
    }

    @inline
    get_group(): i32 {
        return this.m_group;
    }

    @inline
    get_vertex_indexes(): StaticArray<i32> {
        return this.m_polygon;
    }

    @inline
    get_vertex_coordinates(): StaticArray<f32> {
        return this.m_vertices;
    }

    @inline
    get_normal(): StaticArray<f32> {
        return this.m_normal;
    }

    @inline
    get_center(): StaticArray<f32> {
        return this.m_center;
    }

    set_group(group_index: i32, group_array: Array<i32>, all_nodes: StaticArray<NavmeshNode>): void {
        if (this.m_group == -1) {
            this.m_group = group_index;
            let neighbor = this.m_neighbor;
            group_array.push(this.m_index);  // <-- recreate array here
            for (let i = 0, len = this.m_neighbor_count; i < len; i++){
                unchecked(all_nodes[neighbor[i]]).set_group(group_index, group_array, all_nodes);
            }
        }
    }

    is_point_inside(p_x: f32, p_y: f32, p_z: f32): bool {
        let vertices = this.m_vertices;
        let normals = this.m_vertex_normals;
        for (let i = 0, len = this.m_length; i < len; i++) {
            let j = i + 1;
            if (j == len) {
                j = 0;
            }
            let v_x = this._cross_x(
                unchecked(vertices[3 * j + 1] - vertices[3 * i + 1]),
                unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                p_y - unchecked(vertices[3 * i + 1]),
                p_z - unchecked(vertices[3 * i + 2])
            );
            let v_y = this._cross_y(
                unchecked(vertices[3 * j + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                p_x - unchecked(vertices[3 * i + 0]),
                p_z - unchecked(vertices[3 * i + 2])
            );
            let v_z = this._cross_z(
                unchecked(vertices[3 * j + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * j + 1] - vertices[3 * i + 1]),
                p_x - unchecked(vertices[3 * i + 0]),
                p_y - unchecked(vertices[3 * i + 1])
            );
            let d = (
                v_x * unchecked(normals[3 * i + 0]) +
                v_y * unchecked(normals[3 * i + 1]) +
                v_z * unchecked(normals[3 * i + 2])
            );
            if (d < 0) {
                return false;
            }
        }
        return true;
    }

    @inline
    _cross_x(a_y: f32, a_z: f32, b_y: f32, b_z: f32): f32 {
        return a_y * b_z - a_z * b_y;
    }

    @inline
    _cross_y(a_x: f32, a_z: f32, b_x: f32, b_z: f32): f32 {
        return a_z * b_x - a_x * b_z;
    }

    @inline
    _cross_z(a_x: f32, a_y: f32, b_x: f32, b_y: f32): f32 {
        return a_x * b_y - a_y * b_x;
    }

    @inline
    _cross(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): StaticArray<f32> {
        let to_return = new StaticArray<f32>(3);
        unchecked(to_return[0] = a_y * b_z - a_z * b_y);
        unchecked(to_return[1] = a_z * b_x - a_x * b_z);
        unchecked(to_return[2] = a_x * b_y - a_y * b_x);
        return to_return;
    }

    _portals_to_string(): string {
        var to_return = "{";
        let keys = this.m_portals.keys();
        for (let i = 0, len = keys.length; i < len; i++) {
            let key = unchecked(keys[i]);
            to_return += (
                key.toString() + ": " +
                this.m_portals.get(key).toString()
            );
            if (i < len - 1) {
                to_return += ", ";
            }
        }
        return to_return + "}";
    }

    to_string(): string {
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

    toString(): string {
        return this.to_string();
    }
}
