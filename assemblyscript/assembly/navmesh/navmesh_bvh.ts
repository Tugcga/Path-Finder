import { NavmeshNode } from "./navmesh_node";
import { clamp, squared_len, log_message } from "../common/utilities";
import { Serializable, SD_TYPE, 
    bool_bytes_length, bool_to_bytes, bool_from_bytes,
    staticarray_f32_bytes_length, staticarray_f32_to_bytes, staticarray_f32_from_bytes, staticarray_f32_from_bytes_expr } from "../common/binary_io";

class AABB extends Serializable {
    x_min: f32 = 0.0;
    y_min: f32 = 0.0;
    z_min: f32 = 0.0;

    x_max: f32 = 0.0;
    y_max: f32 = 0.0;
    z_max: f32 = 0.0;

    toString(): string {
        return (
            this.x_min.toString() + "," +
            this.y_min.toString() + "," +
            this.z_min.toString() + "," +
            this.x_max.toString() + "," +
            this.y_max.toString() + "," +
            this.z_max.toString()
        );
    }

    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        view.setInt32(0, SD_TYPE.SD_TYPE_AABB);
        view.setInt32(4, bytes_length);
        view.setFloat32(8, this.x_min);
        view.setFloat32(12, this.y_min);
        view.setFloat32(16, this.z_min);

        view.setFloat32(20, this.x_max);
        view.setFloat32(24, this.y_max);
        view.setFloat32(28, this.z_max);
        return to_return;
    }

    override from_bytes(view: DataView, start: u32): void {
        this.x_min = view.getFloat32(start + 8);
        this.y_min = view.getFloat32(start + 12);
        this.z_min = view.getFloat32(start + 16);

        this.x_max = view.getFloat32(start + 20);
        this.y_max = view.getFloat32(start + 24);
        this.z_max = view.getFloat32(start + 28);
    }

    override bytes_length(): u32 {
        return 4  // id
             + 4  // bytes length
             + 4 * 6;  // value
    }
}

function find_node(array: StaticArray<NavmeshNode>, value: i32): NavmeshNode {
    let left = 0;
    let right = array.length - 1;
    while(right - left > 1){
        const mid = (left + right) / 2
        const a_idx = array[mid].get_index();
        if(a_idx == value) {
            return array[mid];
        }

        if(a_idx > value) {
            right = mid;
        }
        else {
            left = mid;
        }
    }
    let a_left = array[left];
    if(array[left].get_index() == value) {
        return a_left;
    }
    let a_right = array[right];
    if(a_right.get_index() == value) {
        return a_right;
    }
    // fail to find the node with the index
    // this is error, but return the first one
    return array[0];
}

export class NavmeshBVH {
    m_aabb: AABB = new AABB();

    m_children_exists: bool;
    m_is_object: bool;

    m_nodes: StaticArray<NavmeshNode>;  // all nodes, come to the current bvh
    m_index_to_node: Map<i32, NavmeshNode>;  // map from node index to this node

    m_left_child!: NavmeshBVH;
    m_right_child!: NavmeshBVH;

    constructor(nodes: StaticArray<NavmeshNode> = new StaticArray<NavmeshNode>(0), BVH_AABB_DELTA: f32 = 0.5) {
        this.m_children_exists = false;
        this.m_is_object = false;
        this.m_nodes = nodes;

        //build map from node index to node
        this.m_index_to_node = new Map<i32, NavmeshNode>();
        for (let i = 0, len = nodes.length; i < len; i++) {
            let node = nodes[i];
            let index = node.get_index();
            this.m_index_to_node.set(index, node);
        }

        let x_min: f32 = Infinity;
        let x_max: f32 = -Infinity;
        let y_min: f32 = Infinity;
        let y_max: f32 = -Infinity;
        let z_min: f32 = Infinity;
        let z_max: f32 = -Infinity;

        if(nodes.length == 0) {
            // input empty array
            this.m_is_object = false;
            this.m_children_exists = false;
        }
        else if (nodes.length == 1) {  // only one object
            this.m_is_object = true;

            //get aabb
            let vertices = this.m_nodes[0].get_vertex_coordinates();
            let verts_count = vertices.length / 3;
            for (let i = 0; i < verts_count; i++) {
                if (vertices[3 * i + 0] < x_min) { x_min = vertices[3 * i + 0]; }
                if (vertices[3 * i + 0] > x_max) { x_max = vertices[3 * i + 0]; }
                if (vertices[3 * i + 1] < y_min) { y_min = vertices[3 * i + 1]; }
                if (vertices[3 * i + 1] > y_max) { y_max = vertices[3 * i + 1]; }
                if (vertices[3 * i + 2] < z_min) { z_min = vertices[3 * i + 2]; }
                if (vertices[3 * i + 2] > z_max) { z_max = vertices[3 * i + 2]; }
            }
            //set aabb
            let aabb = this.m_aabb;
            aabb.x_min = x_min - BVH_AABB_DELTA;
            aabb.y_min = y_min - BVH_AABB_DELTA;
            aabb.z_min = z_min - BVH_AABB_DELTA;

            aabb.x_max = x_max + BVH_AABB_DELTA;
            aabb.y_max = y_max + BVH_AABB_DELTA;
            aabb.z_max = z_max + BVH_AABB_DELTA;
        } else {  // there are many objects, create left and right children

            let x_median: f32 = 0.0;
            let z_median: f32 = 0.0;
            let nodes_len = nodes.length;

            for (let i = 0; i < nodes_len; i++) {
                let node = nodes[i];
                let c = node.get_center();
                let c0 = c[0];
                let c2 = c[2];
                x_median += c0;
                z_median += c2;
                if (c0 < x_min) { x_min = c0; }
                if (c0 > x_max) { x_max = c0; }
                if (c2 < z_min) { z_min = c2; }
                if (c2 > z_max) { z_max = c2; }
            }
            let split_axis = x_max - x_min < z_max - z_min ? 2 : 0;
            let median = (split_axis == 0 ? x_median : z_median) / <f32>nodes_len;

            //reserve arrays for all nodes
            let left  = new StaticArray<NavmeshNode>(nodes_len);
            let right = new StaticArray<NavmeshNode>(nodes_len);
            let left_count  = 0;
            let right_count = 0;

            for (let i = 0; i < nodes_len; i++) {
                let node = nodes[i];
                let c = node.get_center();
                if (split_axis == 0) {
                    if (c[0] < median) {
                        left[left_count] = node;
                        left_count++;
                    } else {
                        right[right_count] = node;
                        right_count++;
                    }
                } else {
                    if (c[2] < median) {
                        left[left_count] = node;
                        left_count++;
                    } else {
                        right[right_count] = node;
                        right_count++;
                    }
                }
            }

            //check that both arrays left and right are non-empty
            if (left_count == 0) {
                left[left_count] = right[right_count - 1];
                ++left_count;
                --right_count;
            } else if (right_count == 0) {
                right[right_count] = left[left_count - 1];
                ++right_count;
                --left_count;
            }

            //next create static arrays and pass it to children nodes
            let left_nodes  = new StaticArray<NavmeshNode>(left_count);
            let right_nodes = new StaticArray<NavmeshNode>(right_count);
            for (let i = 0; i < left_count; i++) {
                left_nodes[i] = left[i];
            }
            for (let i = 0; i < right_count; i++) {
                right_nodes[i] = right[i];
            }

            this.m_left_child  = new NavmeshBVH(left_nodes, BVH_AABB_DELTA);
            this.m_right_child = new NavmeshBVH(right_nodes, BVH_AABB_DELTA);
            this.m_children_exists = true;

            //finally, set aabb
            let left_aabb  = this.m_left_child.get_aabb();
            let right_aabb = this.m_right_child.get_aabb();

            let aabb = this.m_aabb;
            aabb.x_min = Mathf.min(left_aabb.x_min, right_aabb.x_min);
            aabb.y_min = Mathf.min(left_aabb.y_min, right_aabb.y_min);
            aabb.z_min = Mathf.min(left_aabb.z_min, right_aabb.z_min);
            aabb.x_max = Mathf.max(left_aabb.x_max, right_aabb.x_max);
            aabb.y_max = Mathf.max(left_aabb.y_max, right_aabb.y_max);
            aabb.z_max = Mathf.max(left_aabb.z_max, right_aabb.z_max);
        }
    }

    @inline
    get_aabb(): AABB {
        return this.m_aabb;
    }

    @inline
    is_inside_aabb(x: f32, y: f32, z: f32): bool {
        let aabb = this.m_aabb;
        return aabb.x_min < x && aabb.x_max > x &&
               aabb.y_min < y && aabb.y_max > y &&
               aabb.z_min < z && aabb.z_max > z;
    }

    sample(x: f32, y: f32, z: f32): i32 {
        if (this.is_inside_aabb(x, y, z)) {
            if (this.m_children_exists) {  // this node does not contains object, but contains children
                //get left and right sample
                let left_sample  = this.m_left_child.sample(x, y, z);
                let right_sample = this.m_right_child.sample(x, y, z);
                if (left_sample == -1) {
                    return right_sample;
                } else {
                    if (right_sample == -1) {
                        return left_sample;
                    } else {  // both samples are non-empty

                        let left_node = this.m_index_to_node.get(left_sample);
                        let l_c = left_node.get_center();
                        let l_n = left_node.get_normal();
                        let l_dist = Mathf.abs(
                            (x - l_c[0]) * l_n[0] +
                            (y - l_c[1]) * l_n[1] +
                            (z - l_c[2]) * l_n[2]
                        );
                        let right_node = this.m_index_to_node.get(right_sample);
                        let r_c = right_node.get_center();
                        let r_n = right_node.get_normal();
                        let r_dist = Mathf.abs(
                            (x - r_c[0]) * r_n[0] +
                            (y - r_c[1]) * r_n[1] +
                            (z - r_c[2]) * r_n[2]
                        );
                        return l_dist < r_dist ? left_sample : right_sample;
                    }
                }
            } else {  // this is the leaf-node, it contains object
                let first_node = this.m_nodes[0];
                if (first_node.is_point_inside(x, y, z)) {
                    return first_node.get_index();
                } else {
                    return -1;
                }
            }
        }
        return -1;
    }

    // for NavmeshBVH we use custom function from_bytes(), because it need already known array of nodes
    // if we will store it in bytes, then we will save one node several times
    // so, instead we will save only node indices (in array) and keys (in the map)
    // and then in from_bytes() method use already known nodes to fill array and the map
    bytes_length(): u32 {
        let to_return = 8;  // id and bytes length
        to_return += this.m_aabb.bytes_length();
        //for left and right node we store 1 byte (true or false) and if it true, then actual data
        to_return += 2 * bool_bytes_length();  // for children_exists and is_object

        // then store only node indices = map keys
        // in the format: count + k0, k1, ...
        to_return += 4 * (1 + this.m_index_to_node.keys().length);

        // if children exists, store both left and right
        if(this.m_children_exists) {
            to_return += this.m_left_child.bytes_length();
            to_return += this.m_right_child.bytes_length();
        }
        return to_return;
    }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        view.setInt32(0, SD_TYPE.SD_TYPE_NAVMESHBVH);
        view.setInt32(4, bytes_length);
        let shift = 8;
        // then store aabb
        to_return.set(this.m_aabb.to_bytes(), shift);
        shift += this.m_aabb.bytes_length();

        // then m_children_exists
        to_return.set(bool_to_bytes(this.m_children_exists), shift);
        shift += bool_bytes_length();
        // and m_is_object
        to_return.set(bool_to_bytes(this.m_is_object), shift);
        shift += bool_bytes_length();

        // then node indices
        let node_indices = this.m_index_to_node.keys();
        view.setInt32(shift, node_indices.length);
        shift += 4;

        for(let i = 0, len = node_indices.length; i < len; i++) {
            view.setInt32(shift, node_indices[i]);
            shift += 4;
        }

        // finally, if children exist, store it at the end of file
        if(this.m_children_exists) {
            // left
            to_return.set(this.m_left_child.to_bytes(), shift);
            shift += this.m_left_child.bytes_length();
            // right
            to_return.set(this.m_right_child.to_bytes(), shift);
            shift += this.m_right_child.bytes_length();
        }

        return to_return;
    }

    from_bytes(view: DataView, start: u32, nodes: StaticArray<NavmeshNode>): void {
        const id = view.getInt32(start + 0);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_NAVMESHBVH) {
            shift += 8;
        } else { return; }

        // get aabb
        const aabb_id = view.getInt32(shift);
        if(aabb_id == SD_TYPE.SD_TYPE_AABB) {
            const aabb_bytes_length = view.getInt32(shift + 4);
            let new_aabb = new AABB();
            new_aabb.from_bytes(view, shift);
            this.m_aabb = new_aabb;
            shift += aabb_bytes_length;
        } else { return; }

        // next two booleans for m_children_exists and m_is_object
        const ch_exists_id = view.getInt32(shift);
        if(ch_exists_id == SD_TYPE.SD_TYPE_BOOL) {
            const ch_exists_bytes_length = view.getInt32(shift + 4);
            this.m_children_exists = view.getUint8(shift + 8) == 1;
            shift += ch_exists_bytes_length;
        } else { return; }
        const is_obj_id = view.getInt32(shift);
        if(is_obj_id == SD_TYPE.SD_TYPE_BOOL) {
            const is_obj_bytes_length = view.getInt32(shift + 4);
            this.m_is_object = view.getUint8(shift + 8) == 1;
            shift += is_obj_bytes_length;
        } else { return; }

        // next read node indices inside this bvh node
        // they are stored as simple ints
        const count = view.getInt32(shift);
        shift += 4;
        this.m_nodes = new StaticArray<NavmeshNode>(count);
        const nodes_length = nodes.length;
        for(let i = 0; i < count; i++) {
            const index = view.getInt32(shift);
            let n = nodes[index];
            this.m_index_to_node.set(index, n);
            this.m_nodes[i] = n;
            shift += 4;
        }

        // next read left and right nodes, if it stored
        if(this.m_children_exists) {
            const left_id = view.getInt32(shift);
            if(left_id == SD_TYPE.SD_TYPE_NAVMESHBVH) {
                const left_bytes_length = view.getInt32(shift + 4);
                let left = new NavmeshBVH(new StaticArray<NavmeshNode>(0));
                left.from_bytes(view, shift, nodes);
                this.m_left_child = left;
                shift += left_bytes_length
            }
            else { return; }
            // the same process for the right child
            const right_id = view.getInt32(shift);
            if(right_id == SD_TYPE.SD_TYPE_NAVMESHBVH) {
                const right_bytes_length = view.getInt32(shift + 4);
                let right = new NavmeshBVH(new StaticArray<NavmeshNode>(0));
                right.from_bytes(view, shift, nodes);
                this.m_right_child = right;
                shift += right_bytes_length
            }
            else { return; }
        }
    }

    to_string(): string {
        let to_return = "<bvh";
        if (this.m_is_object) {
            to_return += " object " + this.m_nodes[0].get_index().toString() +
                         ", aabb: " + this.m_aabb.toString() +
                         ">";
        } else if(this.m_children_exists) {
            to_return += " left: " + this.m_left_child.to_string() +
                         ", right: " + this.m_right_child.to_string() +
                         ", aabb: " + this.m_aabb.toString() +
                         ">";
        }
        return to_return;
    }

    toString(): string {
        return this.to_string();
    }
}

export class TrianglesBVH extends Serializable {
    //coordinates of the triangle
    m_triangle_data: StaticArray<f32>;
    m_is_object: bool;  // true, if it contains the triangle
    m_children_exists: bool;

    m_aabb: AABB = new AABB();

    m_left_child!: TrianglesBVH;
    m_right_child!: TrianglesBVH;

    m_return_buffer: StaticArray<f32>;  // use this array to return values from sample command

    constructor(triangles_vertices: StaticArray<f32> = new StaticArray<f32>(0), BVH_AABB_DELTA: f32 = 0.5) {
        super();
        this.m_return_buffer = new StaticArray<f32>(4);
        this.m_triangle_data = new StaticArray<f32>(0);
        this.m_is_object = false;
        this.m_children_exists = false;

        if(triangles_vertices.length > 0) {
            if (triangles_vertices.length == 9) {  // 9 values mean that input are one triangle (with three vertices)
                this.m_triangle_data = new StaticArray<f32>(13);
            } else {
                this.m_triangle_data = new StaticArray<f32>(0);
            }

            if (triangles_vertices.length == 9) {  // this is one triangle, then form the object inside the node
                //copy triangle data
                //https://www.gamedev.net/forums/topic/552906-closest-point-on-triangle/
                //store first point

                let triangle_data = this.m_triangle_data;

                triangle_data[0] = triangles_vertices[0];
                triangle_data[1] = triangles_vertices[1];
                triangle_data[2] = triangles_vertices[2];
                //edge e1 = v0->v1
                triangle_data[3] = triangles_vertices[3] - triangles_vertices[0];
                triangle_data[4] = triangles_vertices[4] - triangles_vertices[1];
                triangle_data[5] = triangles_vertices[5] - triangles_vertices[2];
                //edge e2 = v0->v2
                triangle_data[6] = triangles_vertices[6] - triangles_vertices[0];
                triangle_data[7] = triangles_vertices[7] - triangles_vertices[1];
                triangle_data[8] = triangles_vertices[8] - triangles_vertices[2];
                //a = (e1, e1)
                triangle_data[9] = squared_len(
                    triangle_data[3],
                    triangle_data[4],
                    triangle_data[5]
                );
                //b = (e1, e2)
                triangle_data[10] = (
                    triangle_data[3] * triangle_data[6] +
                    triangle_data[4] * triangle_data[7] +
                    triangle_data[5] * triangle_data[8]
                );
                //c = (e2, e2)
                triangle_data[11] = squared_len(
                    triangle_data[6],
                    triangle_data[7],
                    triangle_data[8]
                );
                //determinant [[a b], [b c]]
                triangle_data[12] = (
                    triangle_data[9]  * triangle_data[11] -
                    triangle_data[10] * triangle_data[10]
                );

                this.m_is_object = true;

                //calculate aabb of the triangle
                let aabb = this.m_aabb;
                aabb.x_min = this._min3(
                    triangles_vertices[0],
                    triangles_vertices[3],
                    triangles_vertices[6]
                );
                aabb.y_min = this._min3(
                    triangles_vertices[1],
                    triangles_vertices[4],
                    triangles_vertices[7]
                );
                aabb.z_min = this._min3(
                    triangles_vertices[2],
                    triangles_vertices[5],
                    triangles_vertices[8]
                );
                aabb.x_max = this._max3(
                    triangles_vertices[0],
                    triangles_vertices[3],
                    triangles_vertices[6]
                );
                aabb.y_max = this._max3(
                    triangles_vertices[1],
                    triangles_vertices[4],
                    triangles_vertices[7]
                );
                aabb.z_max = this._max3(
                    triangles_vertices[2],
                    triangles_vertices[5],
                    triangles_vertices[8]
                );

                this._extend_aabb_by_delta(BVH_AABB_DELTA);
            } else {
                var median_x: f32 = 0.0;
                var median_z: f32 = 0.0;
                let min_x: f32 =  Infinity;
                let min_y: f32 =  Infinity;
                let min_z: f32 =  Infinity;
                let max_x: f32 = -Infinity;
                let max_y: f32 = -Infinity;
                let max_z: f32 = -Infinity;

                var objects_count = triangles_vertices.length / 9;

                for (let i = 0; i < objects_count; i++) {
                    min_x = this._min4(
                        min_x,
                        triangles_vertices[9 * i + 0],
                        triangles_vertices[9 * i + 3],
                        triangles_vertices[9 * i + 6]
                    );
                    min_y = this._min4(
                        min_y,
                        triangles_vertices[9 * i + 1],
                        triangles_vertices[9 * i + 4],
                        triangles_vertices[9 * i + 7]
                    );
                    min_z = this._min4(
                        min_z,
                        triangles_vertices[9 * i + 2],
                        triangles_vertices[9 * i + 5],
                        triangles_vertices[9 * i + 8]
                    );

                    max_x = this._max4(
                        max_x,
                        triangles_vertices[9 * i + 0],
                        triangles_vertices[9 * i + 3],
                        triangles_vertices[9 * i + 6]
                    );
                    max_y = this._max4(
                        max_y,
                        triangles_vertices[9 * i + 1],
                        triangles_vertices[9 * i + 4],
                        triangles_vertices[9 * i + 7]
                    );
                    max_z = this._max4(
                        max_z,
                        triangles_vertices[9 * i + 2],
                        triangles_vertices[9 * i + 5],
                        triangles_vertices[9 * i + 8]
                    );

                    median_x += triangles_vertices[9 * i + 0];
                    median_x += triangles_vertices[9 * i + 3];
                    median_x += triangles_vertices[9 * i + 6];

                    median_z += triangles_vertices[9 * i + 2];
                    median_z += triangles_vertices[9 * i + 5];
                    median_z += triangles_vertices[9 * i + 8];
                }

                let aabb = this.m_aabb;
                aabb.x_min = min_x;
                aabb.y_min = min_y;
                aabb.z_min = min_z;
                aabb.x_max = max_x;
                aabb.y_max = max_y;
                aabb.z_max = max_z;

                this._extend_aabb_by_delta(BVH_AABB_DELTA);

                const objects_count_int = objects_count * 3;
                let total: f32 = 1.0 / <f32>objects_count_int;
                median_x *= total;
                median_z *= total;

                let axis: i8 = this._get_aabb_x_size() > this._get_aabb_z_size() ? 0 : 2;  // 0 - axis parallel to z, 2 - parallel to x
                //next we shold enumerate all objects and devide it into two parts - left and right
                //create two full buffers
                let left_objects  = new StaticArray<f32>(9 * objects_count);
                let right_objects = new StaticArray<f32>(9 * objects_count);
                let left_count  = 0;
                let right_count = 0;

                for (let i = 0; i < objects_count; i++) {
                    //get the center of the triangle
                    let c_x: f32 = (
                        triangles_vertices[9 * i + 0] +
                        triangles_vertices[9 * i + 3] +
                        triangles_vertices[9 * i + 6]
                    ) / 3.0;

                    let c_z: f32 = (
                        triangles_vertices[9 * i + 2] +
                        triangles_vertices[9 * i + 5] +
                        triangles_vertices[9 * i + 8]
                    ) / 3.0;

                    if ((axis == 0 && c_x < median_x) || (axis == 2 && c_z < median_z)) { //add to the left
                        left_objects[9 * left_count + 0] = triangles_vertices[9 * i + 0];
                        left_objects[9 * left_count + 1] = triangles_vertices[9 * i + 1];
                        left_objects[9 * left_count + 2] = triangles_vertices[9 * i + 2];

                        left_objects[9 * left_count + 3] = triangles_vertices[9 * i + 3];
                        left_objects[9 * left_count + 4] = triangles_vertices[9 * i + 4];
                        left_objects[9 * left_count + 5] = triangles_vertices[9 * i + 5];

                        left_objects[9 * left_count + 6] = triangles_vertices[9 * i + 6];
                        left_objects[9 * left_count + 7] = triangles_vertices[9 * i + 7];
                        left_objects[9 * left_count + 8] = triangles_vertices[9 * i + 8];

                        left_count++;
                    } else {//add to the right
                        right_objects[9 * right_count + 0] = triangles_vertices[9 * i + 0];
                        right_objects[9 * right_count + 1] = triangles_vertices[9 * i + 1];
                        right_objects[9 * right_count + 2] = triangles_vertices[9 * i + 2];

                        right_objects[9 * right_count + 3] = triangles_vertices[9 * i + 3];
                        right_objects[9 * right_count + 4] = triangles_vertices[9 * i + 4];
                        right_objects[9 * right_count + 5] = triangles_vertices[9 * i + 5];

                        right_objects[9 * right_count + 6] = triangles_vertices[9 * i + 6];
                        right_objects[9 * right_count + 7] = triangles_vertices[9 * i + 7];
                        right_objects[9 * right_count + 8] = triangles_vertices[9 * i + 8];

                        right_count++;
                    }
                }

                //check non-infinite recursion
                if (left_count > 0 && right_count == 0) {  // move last left object to the right
                    right_objects[0] = left_objects[(left_count - 1) * 9 + 0];
                    right_objects[1] = left_objects[(left_count - 1) * 9 + 1];
                    right_objects[2] = left_objects[(left_count - 1) * 9 + 2];

                    right_objects[3] = left_objects[(left_count - 1) * 9 + 3];
                    right_objects[4] = left_objects[(left_count - 1) * 9 + 4];
                    right_objects[5] = left_objects[(left_count - 1) * 9 + 5];

                    right_objects[6] = left_objects[(left_count - 1) * 9 + 6];
                    right_objects[7] = left_objects[(left_count - 1) * 9 + 7];
                    right_objects[8] = left_objects[(left_count - 1) * 9 + 8];

                    right_count++;
                    left_count--;
                } else if (left_count == 0 && right_count > 0) {  // move last right object to the left

                    left_objects[0] = right_objects[(right_count - 1) * 9 + 0];
                    left_objects[1] = right_objects[(right_count - 1) * 9 + 1];
                    left_objects[2] = right_objects[(right_count - 1) * 9 + 2];

                    left_objects[3] = right_objects[(right_count - 1) * 9 + 3];
                    left_objects[4] = right_objects[(right_count - 1) * 9 + 4];
                    left_objects[5] = right_objects[(right_count - 1) * 9 + 5];

                    left_objects[6] = right_objects[(right_count - 1) * 9 + 6];
                    left_objects[7] = right_objects[(right_count - 1) * 9 + 7];
                    left_objects[8] = right_objects[(right_count - 1) * 9 + 8];

                    left_count++;
                    right_count--;
                }

                //finally, create two child nodes
                //create typed arrays and fill it
                var left_array  = new StaticArray<f32>(left_count * 9);
                var right_array = new StaticArray<f32>(right_count * 9);

                for (let j = 0, len = left_count * 9; j < len; j++) {
                    left_array[j] = left_objects[j];
                }

                for (let j = 0, len = right_count * 9; j < len; j++) {
                    right_array[j] = right_objects[j];
                }

                this.m_left_child  = new TrianglesBVH(left_array, BVH_AABB_DELTA);
                this.m_right_child = new TrianglesBVH(right_array, BVH_AABB_DELTA);
                this.m_children_exists = true;
            }
        }
    }

    @inline
    _min3(a: f32, b: f32, c: f32): f32 {
        let t = Mathf.min(a, b);
        return Mathf.min(t, c);
    }

    @inline
    _max3(a: f32, b: f32, c: f32): f32 {
        let t = Mathf.max(a, b);
        return Mathf.max(t, c);
    }

    @inline
    _min4(a: f32, b: f32, c: f32, d: f32): f32 {
        let u = Mathf.min(a, b);
        let v = Mathf.min(c, d);
        return Mathf.min(u, v);
    }

    @inline
    _max4(a: f32, b: f32, c: f32, d: f32): f32 {
        let u = Mathf.max(a, b);
        let v = Mathf.max(c, d);
        return Mathf.max(u, v);
    }

    @inline
    _get_aabb_x_size(): f32 {
        let aabb = this.m_aabb;
        return aabb.x_max - aabb.x_min;
    }

    @inline
    _get_aabb_y_size(): f32 {
        let aabb = this.m_aabb;
        return aabb.y_max - aabb.y_min;
    }

    @inline
    _get_aabb_z_size(): f32 {
        let aabb = this.m_aabb;
        return aabb.z_max - aabb.z_min;
    }

    @inline
    _extend_aabb_by_delta(delta: f32): void {
        let aabb = this.m_aabb;
        aabb.x_min -= delta;
        aabb.y_min -= delta;
        aabb.z_min -= delta;

        aabb.x_max += delta;
        aabb.y_max += delta;
        aabb.z_max += delta;
    }

    @inline
    _get_aabb(): AABB {
        return this.m_aabb;
    }

    @inline
    _is_inside_aabb(x: f32, y: f32, z: f32): bool {
        let aabb = this.m_aabb;
        return (
            x > aabb.x_min && x < aabb.x_max &&
            y > aabb.y_min && y < aabb.y_max &&
            z > aabb.z_min && z < aabb.z_max
        );
    }

    sample(x: f32, y: f32, z: f32): StaticArray<f32> {
        let triangle_data = this.m_triangle_data;
        //return the 4-th [x, y, z, w], where w = 1.0 - correct answer, 0.0 - empty answer
        if (this._is_inside_aabb(x, y, z)) {  // point inside aabb, so, check the node
            if (this.m_is_object) {  // this node contains object, so, return actual closest position in the triangle
                //here we should find actual closest point
                let v0_x = triangle_data[0] - x;
                let v0_y = triangle_data[1] - y;
                let v0_z = triangle_data[2] - z;

                //d = (e1, v0)
                let d = (
                    triangle_data[3] * v0_x +
                    triangle_data[4] * v0_y +
                    triangle_data[5] * v0_z
                );
                //e = (e2, v0)
                let e = (
                    triangle_data[6] * v0_x +
                    triangle_data[7] * v0_y +
                    triangle_data[8] * v0_z
                );

                //s = b*e - c*d
                //t = b*d - a*e
                let s = (
                    triangle_data[10] * e -
                    triangle_data[11] * d
                );
                let t = (
                    triangle_data[10] * d -
                    triangle_data[9]  * e
                );

                //det = this.triangle_data[12]
                //a = this.triangle_data[9]
                //b = this.triangle_data[10]
                //c = this.triangle_data[11]
                if (s + t < triangle_data[12]) {
                    if (s < 0) {
                        if (t < 0) {
                            if (d < 0) {
                                s = clamp(-d / triangle_data[9]);
                                t = 0;
                            } else {
                                s = 0;
                                t = clamp(-e / triangle_data[11]);
                            }
                        } else {
                            s = 0;
                            t = clamp(-e / triangle_data[11]);
                        }
                    } else if (t < 0) {
                        s = clamp(-d / triangle_data[9]);
                        t = 0.0;
                    } else {
                        let invDet: f32 = 1.0 / triangle_data[12];
                        s *= invDet;
                        t *= invDet;
                    }
                } else {
                    if (s < 0) {
                        let tmp0 = triangle_data[10] + d;
                        let tmp1 = triangle_data[11] + e;
                        if (tmp1 > tmp0) {
                            let numer = tmp1 - tmp0;
                            let denom = triangle_data[9] - 2 * triangle_data[10] + triangle_data[11];
                            s = clamp(numer / denom);
                            t = 1.0 - s;
                        } else {
                            t = clamp(-e / triangle_data[11]);
                            s = 0;
                        }
                    } else if (t < 0) {
                        if (triangle_data[9] + d > triangle_data[10] + e) {
                            let numer = triangle_data[11] + e - triangle_data[10] - d;
                            let denom = triangle_data[9]  - 2 * triangle_data[10] + triangle_data[11];
                            s = clamp(numer / denom);
                            t = 1.0 -  s;
                        } else {
                            s = clamp(-d / triangle_data[9]);
                            t = 0;
                        }
                    } else {
                        let numer = triangle_data[11] + e - triangle_data[10] - d;
                        let denom = triangle_data[9] - 2 * triangle_data[10] + triangle_data[11];
                        s = clamp(numer / denom);
                        t = 1.0 - s;
                    }
                }

                let return_buffer = this.m_return_buffer;
                return_buffer[0] = triangle_data[0] + s * triangle_data[3] + t * triangle_data[6];
                return_buffer[1] = triangle_data[1] + s * triangle_data[4] + t * triangle_data[7];
                return_buffer[2] = triangle_data[2] + s * triangle_data[5] + t * triangle_data[8];
                return_buffer[3] = 1.0;

                return return_buffer;
            } else {  // node contains children, check it

                let left_sample  = this.m_left_child.sample(x, y, z);
                let right_sample = this.m_right_child.sample(x, y, z);

                if (left_sample[3] < 0.5) {
                    return right_sample;
                } else {
                    if (right_sample[3] < 0.5) {
                        return left_sample;
                    } else {
                        //both left and right sample is correct, so, return the closest to the initial point
                        let d_l = squared_len(
                            x - left_sample[0],
                            y - left_sample[1],
                            z - left_sample[2]
                        );
                        let d_r = squared_len(
                            x - right_sample[0],
                            y - right_sample[1],
                            z - right_sample[2]
                        );
                        return d_l < d_r ? left_sample : right_sample;
                    }
                }
            }
        } else {  // point outside the aabb, so, skip next traversing, return false answer
            this.m_return_buffer[3] = 0.0;
            return this.m_return_buffer;
        }
    }

    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        view.setInt32(0, SD_TYPE.SD_TYPE_TRIANGLESBVH);
        view.setInt32(4, bytes_length);

        let shift = 8;
        // save triangles data
        to_return.set(staticarray_f32_to_bytes(this.m_triangle_data), shift);
        shift += staticarray_f32_bytes_length(this.m_triangle_data);

        // next two bools
        to_return.set(bool_to_bytes(this.m_is_object), shift);
        shift += bool_bytes_length();
        to_return.set(bool_to_bytes(this.m_children_exists), shift);
        shift += bool_bytes_length();

        // next aabb
        to_return.set(this.m_aabb.to_bytes(), shift);
        shift += this.m_aabb.bytes_length();

        // children, if exits
        if(this.m_children_exists) {
            // left
            to_return.set(this.m_left_child.to_bytes(), shift);
            shift += this.m_left_child.bytes_length();

            // right
            to_return.set(this.m_right_child.to_bytes(), shift);
            shift += this.m_right_child.bytes_length();
        }
        // does not store m_return_buffer
        return to_return;
    }

    override from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_TRIANGLESBVH) {
            shift += 8;
        } else { return; }

        // read triangles data array
        const data_id = view.getInt32(shift);
        if(data_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const data_bytes_length = view.getInt32(shift + 4);
            this.m_triangle_data = staticarray_f32_from_bytes_expr(view, shift);
            shift += data_bytes_length;
        } else { return; }

        // next read two bools
        // m_is_object
        const is_object_id = view.getInt32(shift);
        if(is_object_id == SD_TYPE.SD_TYPE_BOOL) {
            const is_object_bytes_length = view.getInt32(shift + 4);
            this.m_is_object = view.getUint8(shift + 8) == 1;
            shift += is_object_bytes_length;
        } else { return; }
        // m_children_exists
        const children_exists_id = view.getInt32(shift);
        if(children_exists_id == SD_TYPE.SD_TYPE_BOOL) {
            const children_exists_bytes_length = view.getInt32(shift + 4);
            this.m_children_exists = view.getUint8(shift + 8) == 1;
            shift += children_exists_bytes_length;
        } else { return; }

        // next read aabb
        const aabb_id = view.getInt32(shift);
        if(aabb_id == SD_TYPE.SD_TYPE_AABB) {
            const aabb_bytes_length = view.getInt32(shift + 4);
            this.m_aabb = new AABB();
            this.m_aabb.from_bytes(view, shift);
            shift += aabb_bytes_length;
        } else { return; }

        // and finally read children
        if(this.m_children_exists) {
            const left_id =view.getInt32(shift);
            if(left_id == SD_TYPE.SD_TYPE_TRIANGLESBVH) {
                const left_bytes_length = view.getInt32(shift + 4);
                this.m_left_child = new TrianglesBVH();
                this.m_left_child.from_bytes(view, shift);
                shift += left_bytes_length;
            } else { return; }

            const right_id =view.getInt32(shift);
            if(right_id == SD_TYPE.SD_TYPE_TRIANGLESBVH) {
                const right_bytes_length = view.getInt32(shift + 4);
                this.m_right_child = new TrianglesBVH();
                this.m_right_child.from_bytes(view, shift);
                shift += right_bytes_length;
            } else { return; }
        }
    }

    override bytes_length(): u32 {
        let to_return = 8;  // id, bytes length
        to_return += staticarray_f32_bytes_length(this.m_triangle_data);
        to_return += bool_bytes_length();  // m_is_object
        to_return += bool_bytes_length();  // m_children_exists
        to_return += this.m_aabb.bytes_length();
        if(this.m_children_exists) {
            to_return += this.m_left_child.bytes_length();
            to_return += this.m_right_child.bytes_length();
        }

        return to_return;
    }

    to_string(): string {
        return "[(" + this._get_aabb().toString() + "): " + (
            this.m_is_object ? "tiangle<>" : "left-"  +
            this.m_left_child.to_string() + " right-" +
            this.m_right_child.to_string()
        ) + "]";
    }

    toString(): string {
        return this.to_string();
    }
}
