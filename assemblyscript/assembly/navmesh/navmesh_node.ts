import { log_message } from "../common/utilities";
import { Serializable, SD_TYPE,
    staticarray_f32_bytes_length, staticarray_f32_to_bytes, staticarray_f32_from_bytes_expr,
    staticarray_i32_bytes_length, staticarray_i32_to_bytes, staticarray_i32_from_bytes_expr,
    i32_bytes_length, i32_to_bytes, i32_from_bytes,
    map_i32_staticarray_f32_bytes_length, map_i32_staticarray_f32_to_bytes, map_i32_staticarray_f32_from_bytes_expr } from "../common/binary_io";

let STATIC_ARRAY_BUFFER_STEP = 8;  // when increase the data values count in static array and overflow it length, then we recreate new bigger array

@inline
function _cross_x(a_y: f32, a_z: f32, b_y: f32, b_z: f32): f32 {
    return a_y * b_z - a_z * b_y;
}

@inline
function _cross_y(a_x: f32, a_z: f32, b_x: f32, b_z: f32): f32 {
    return a_z * b_x - a_x * b_z;
}

@inline
function _cross_z(a_x: f32, a_y: f32, b_x: f32, b_y: f32): f32 {
    return a_x * b_y - a_y * b_x;
}

@inline
function _cross(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): StaticArray<f32> {
    let to_return = new StaticArray<f32>(3);
    unchecked(to_return[0] = a_y * b_z - a_z * b_y);
    unchecked(to_return[1] = a_z * b_x - a_x * b_z);
    unchecked(to_return[2] = a_x * b_y - a_y * b_x);
    return to_return;
}

export class NavmeshNode extends Serializable {
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

    constructor(all_vertices: StaticArray<f32> = new StaticArray<f32>(0), 
                index: i32 = -1, 
                polygon_indexes: StaticArray<i32> = new StaticArray<i32>(0)) {
        // pass index = -1 for empty inicialization (for load from binary later)
        super();
        if(index == -1) {
            this.m_vertices = new StaticArray<f32>(0);
            this.m_polygon = new StaticArray<i32>(0);
            this.m_length = 0;
            this.m_index = -1;
            this.m_group = -1;
            this.m_neighbor = new StaticArray<i32>(0);
            this.m_neighbor_count = 0;
            this.m_center = new StaticArray<f32>(0);
            this.m_normal = new StaticArray<f32>(0);
            this.m_vertex_normals = new StaticArray<f32>(0);
            this.m_portals = new Map<i32, StaticArray<f32>>();
        }
        else {
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
                let c_x = _cross_x(
                    unchecked(vertices[3 * i + 1]) - cy,
                    unchecked(vertices[3 * i + 2]) - cz,
                    unchecked(vertices[3 * j + 1]) - cy,
                    unchecked(vertices[3 * j + 2]) - cz
                );
                let c_y = _cross_y(
                    unchecked(vertices[3 * i + 0]) - cx,
                    unchecked(vertices[3 * i + 2]) - cz,
                    unchecked(vertices[3 * j + 0]) - cx,
                    unchecked(vertices[3 * j + 2]) - cz
                );
                let c_z = _cross_z(
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
                let inv_len: f32 = 1.0 / Mathf.sqrt(nx * nx + ny * ny + nz * nz);
                unchecked(this.m_normal[0] = nx * inv_len);
                unchecked(this.m_normal[1] = ny * inv_len);
                unchecked(this.m_normal[2] = nz * inv_len);
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
                let v_x = _cross_x(
                    unchecked(vertices[3 * j + 1] - vertices[3 * i + 1]),
                    unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                    unchecked(vertices[3 * k + 1] - vertices[3 * i + 1]),
                    unchecked(vertices[3 * k + 2] - vertices[3 * i + 2])
                );
                let v_y = _cross_y(
                    unchecked(vertices[3 * j + 0] - vertices[3 * i + 0]),
                    unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                    unchecked(vertices[3 * k + 0] - vertices[3 * i + 0]),
                    unchecked(vertices[3 * k + 2] - vertices[3 * i + 2])
                );
                let v_z = _cross_z(
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
                neighbor = new_neighbor;
            }

            this.m_neighbor_count++;
            unchecked(neighbor[this.m_neighbor_count - 1] = node_index);
        }
    }

    @inline
    get_polygon(): StaticArray<i32>{
        return this.m_polygon;
    }

    @inline
    get_normal_y(): f32{
        return this.m_normal[1];
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
            for (let i = 0, len = this.m_neighbor_count; i < len; i++) {
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
            let v_x = _cross_x(
                unchecked(vertices[3 * j + 1] - vertices[3 * i + 1]),
                unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                p_y - unchecked(vertices[3 * i + 1]),
                p_z - unchecked(vertices[3 * i + 2])
            );
            let v_y = _cross_y(
                unchecked(vertices[3 * j + 0] - vertices[3 * i + 0]),
                unchecked(vertices[3 * j + 2] - vertices[3 * i + 2]),
                p_x - unchecked(vertices[3 * i + 0]),
                p_z - unchecked(vertices[3 * i + 2])
            );
            let v_z = _cross_z(
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

            if (d < -0.00001) {
                return false;
            }
        }
        return true;
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

    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_NAVMESHNODE);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // vertices
        let vertices = staticarray_f32_to_bytes(this.m_vertices);
        to_return.set(vertices, shift);
        shift += staticarray_f32_bytes_length(this.m_vertices);

        // polygon
        let polygon = staticarray_i32_to_bytes(this.m_polygon);
        to_return.set(polygon, shift);
        shift += staticarray_i32_bytes_length(this.m_polygon);

        // length
        let length = i32_to_bytes(this.m_length);
        to_return.set(length, shift);
        shift += i32_bytes_length();

        // index
        let index = i32_to_bytes(this.m_index);
        to_return.set(index, shift);
        shift += i32_bytes_length();

        // group
        let group = i32_to_bytes(this.m_group);
        to_return.set(group, shift);
        shift += i32_bytes_length();

        // neighbor
        let neighbor = staticarray_i32_to_bytes(this.m_neighbor);
        to_return.set(neighbor, shift);
        shift += staticarray_i32_bytes_length(this.m_neighbor);

        // neighbor count
        let neighbor_count = i32_to_bytes(this.m_neighbor_count);
        to_return.set(neighbor_count, shift);
        shift += i32_bytes_length();

        // center
        let center = staticarray_f32_to_bytes(this.m_center);
        to_return.set(center, shift);
        shift += staticarray_f32_bytes_length(this.m_center);

        // normal
        let normal = staticarray_f32_to_bytes(this.m_normal);
        to_return.set(normal, shift);
        shift += staticarray_f32_bytes_length(this.m_normal);

        // vertex normals
        let v_normals = staticarray_f32_to_bytes(this.m_vertex_normals);
        to_return.set(v_normals, shift);
        shift += staticarray_f32_bytes_length(this.m_vertex_normals);

        // portals
        let portals = map_i32_staticarray_f32_to_bytes(this.m_portals);
        to_return.set(portals, shift);

        return to_return;
    }

    override from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_NAVMESHNODE) {
            shift += 8;
        } else { return; }

        const vertices_id = view.getInt32(shift);
        if(vertices_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const vertices_bytes_length = view.getInt32(shift + 4);
            this.m_vertices = staticarray_f32_from_bytes_expr(view, shift);
            shift += vertices_bytes_length;
        } else { return; }

        const polygon_id = view.getInt32(shift);
        if(polygon_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
            const polygon_bytes_length = view.getInt32(shift + 4);
            this.m_polygon = staticarray_i32_from_bytes_expr(view, shift);
            shift += polygon_bytes_length;
        } else { return; }

        const length_id = view.getInt32(shift);
        if(length_id == SD_TYPE.SD_TYPE_INT32) {
            const length_bytes_length = view.getInt32(shift + 4);
            this.m_length = view.getInt32(shift + 8);
            shift += length_bytes_length;
        } else { return; }

        const index_id = view.getInt32(shift);
        if(index_id == SD_TYPE.SD_TYPE_INT32) {
            const index_bytes_length = view.getInt32(shift + 4);
            this.m_index = view.getInt32(shift + 8);
            shift += index_bytes_length;
        } else { return; }

        const group_id = view.getInt32(shift);
        if(group_id == SD_TYPE.SD_TYPE_INT32) {
            const group_bytes_length = view.getInt32(shift + 4);
            this.m_group = view.getInt32(shift + 8);
            shift += group_bytes_length;
        } else { return; }

        const neighbor_id = view.getInt32(shift);
        if(neighbor_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
            const neighbor_bytes_length = view.getInt32(shift + 4);
            this.m_neighbor = staticarray_i32_from_bytes_expr(view, shift);
            shift += neighbor_bytes_length;
        } else { return; }

        const neighbor_count_id = view.getInt32(shift);
        if(neighbor_count_id == SD_TYPE.SD_TYPE_INT32) {
            const neighbor_count_bytes_length = view.getInt32(shift + 4);
            this.m_neighbor_count = view.getInt32(shift + 8);
            shift += neighbor_count_bytes_length;
        } else { return; }

        const center_id = view.getInt32(shift);
        if(center_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const center_bytes_length = view.getInt32(shift + 4);
            this.m_center = staticarray_f32_from_bytes_expr(view, shift);
            shift += center_bytes_length;
        } else { return; }

        const normal_id = view.getInt32(shift);
        if(normal_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const normal_bytes_length = view.getInt32(shift + 4);
            this.m_normal = staticarray_f32_from_bytes_expr(view, shift);
            shift += normal_bytes_length;
        } else { return; }

        const v_normals_id = view.getInt32(shift);
        if(v_normals_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const v_normals_bytes_length = view.getInt32(shift + 4);
            this.m_vertex_normals = staticarray_f32_from_bytes_expr(view, shift);
            shift += v_normals_bytes_length;
        } else { return; }

        const portals_id = view.getInt32(shift);
        if(portals_id == SD_TYPE.SD_TYPE_MAP_INT32_STATICARRAY_FLOAT32) {
            const portals_bytes_length = view.getInt32(shift + 4);
            this.m_portals = map_i32_staticarray_f32_from_bytes_expr(view, shift);
            shift += portals_bytes_length;
        } else { return; }
    }

    override bytes_length(): u32 {
        return 4  // id
             + 4  // bytes length
             + staticarray_f32_bytes_length(this.m_vertices)
             + staticarray_i32_bytes_length(this.m_polygon)
             + i32_bytes_length()  // m_length
             + i32_bytes_length()  // m_index
             + i32_bytes_length()  // m_group
             + staticarray_i32_bytes_length(this.m_neighbor)
             + i32_bytes_length()  // m_neighbor_count
             + staticarray_f32_bytes_length(this.m_center)
             + staticarray_f32_bytes_length(this.m_normal)
             + staticarray_f32_bytes_length(this.m_vertex_normals)
             + map_i32_staticarray_f32_bytes_length(this.m_portals);
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

// StaticArray<NavmeshNode>
export function staticarray_navmeshnode_bytes_length(nodes: StaticArray<NavmeshNode>): u32 {
    let to_return = 12;  // id, bytes length, count
    for(let i = 0, len = nodes.length; i < len; i++) {
        let node = nodes[i];
        to_return += node.bytes_length();
    }

    return to_return;
}

export function staticarray_navmeshnode_to_bytes(nodes: StaticArray<NavmeshNode>): Uint8Array {
    const bytes_length = staticarray_navmeshnode_bytes_length(nodes);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_STATICARRAY_NAVMESHNODE);
    view.setInt32(4, bytes_length);
    view.setInt32(8, nodes.length);
    let shift = 12;
    for(let i = 0, len = nodes.length; i < len; i++) {
        let node = nodes[i];
        const node_bytes_length = node.bytes_length();
        to_return.set(node.to_bytes(), shift);
        shift += node_bytes_length;
    }
    return to_return;
}

export function staticarray_navmeshnode_from_bytes(bytes: Uint8Array): StaticArray<NavmeshNode> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        const id = view.getInt32(0);
        const bytes_length = view.getInt32(4);
        if(id == SD_TYPE.SD_TYPE_STATICARRAY_NAVMESHNODE) {
            const count = view.getInt32(8);
            let shift = 12;
            let to_return = new StaticArray<NavmeshNode>(count);
            for(let i = 0; i < count; i++) {
                const nn_bytes_length = view.getInt32(shift + 4);
                let node = new NavmeshNode();
                node.from_bytes(bytes.slice(shift, shift + nn_bytes_length));
                to_return[i] = node;
                shift += nn_bytes_length;
            }

            return to_return;
        }
        else {
            return new StaticArray<NavmeshNode>(0);
        }
    }
    else {
        return new StaticArray<NavmeshNode>(0);
    }
}

export function staticarray_navmeshnode_from_bytes_expr(view: DataView, start: u32): StaticArray<NavmeshNode> {
    const count = view.getInt32(start + 8);
    let shift = start + 12;
    let to_return = new StaticArray<NavmeshNode>(count);
    for(let i = 0; i < count; i++) {
        const nn_bytes_length = view.getInt32(shift + 4);
        let node = new NavmeshNode();
        node.from_bytes(view, shift);
        to_return[i] = node;
        shift += nn_bytes_length;
    }

    return to_return;
}
