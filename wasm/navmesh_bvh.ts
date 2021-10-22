import { NavmeshNode } from "./navmesh_node";

export class INavmeshBVH {
    m_aabb: StaticArray<f32>;
    m_children: StaticArray<INavmeshBVH>;
    m_children_exists: bool;
    m_is_object: bool;
    m_nodes: StaticArray<NavmeshNode>;  // all nodes, come to the current bvh
    m_index_to_node: Map<i32, NavmeshNode>;  // map from node index to this node

    constructor(nodes: StaticArray<NavmeshNode>, BVH_AABB_DELTA: f32 = 0.5) {
        this.m_aabb = new StaticArray<f32>(6);
        this.m_children = new StaticArray<INavmeshBVH>(2);
        this.m_children_exists = false;
        this.m_is_object = false;
        this.m_nodes = nodes;

        //build map from node index to node
        this.m_index_to_node = new Map<i32, NavmeshNode>();
        for (let i = 0, len = nodes.length; i < len; i++) {
            let node = unchecked(nodes[i]);
            let index = node.get_index();
            this.m_index_to_node.set(index, node);
        }

        let x_min: f32 = Infinity;
        let x_max: f32 = -Infinity;
        let y_min: f32 = Infinity;
        let y_max: f32 = -Infinity;
        let z_min: f32 = Infinity;
        let z_max: f32 = -Infinity;

        if (nodes.length == 1) {  // only one object, so, but it to the current bvh node
            //this.m_object = nodes[0];  // reassign object link
            this.m_is_object = true;

            //get aabb
            //let vertices: StaticArray<f32> = this.m_object.get_vertex_coordinates();
            let vertices = unchecked(this.m_nodes[0]).get_vertex_coordinates();
            let verts_count = vertices.length / 3;
            for (let i = 0; i < verts_count; i++) {
                if (unchecked(vertices[3 * i + 0]) < x_min) { x_min = unchecked(vertices[3 * i + 0]); }
                if (unchecked(vertices[3 * i + 0]) > x_max) { x_max = unchecked(vertices[3 * i + 0]); }
                if (unchecked(vertices[3 * i + 1]) < y_min) { y_min = unchecked(vertices[3 * i + 1]); }
                if (unchecked(vertices[3 * i + 1]) > y_max) { y_max = unchecked(vertices[3 * i + 1]); }
                if (unchecked(vertices[3 * i + 2]) < z_min) { z_min = unchecked(vertices[3 * i + 2]); }
                if (unchecked(vertices[3 * i + 2]) > z_max) { z_max = unchecked(vertices[3 * i + 2]); }
            }
            //set aabb
            unchecked(this.m_aabb[0] = x_min - BVH_AABB_DELTA);
            unchecked(this.m_aabb[1] = y_min - BVH_AABB_DELTA);
            unchecked(this.m_aabb[2] = z_min - BVH_AABB_DELTA);
            unchecked(this.m_aabb[3] = x_max + BVH_AABB_DELTA);
            unchecked(this.m_aabb[4] = y_max + BVH_AABB_DELTA);
            unchecked(this.m_aabb[5] = z_max + BVH_AABB_DELTA);
        } else {  // there are many objects, create left and right children

            let x_median: f32 = 0.0;
            let z_median: f32 = 0.0;
            let nodesLen = nodes.length;

            for (let i = 0; i < nodesLen; i++) {
                let node = unchecked(nodes[i]);
                let c = node.get_center();
                let c0 = unchecked(c[0]);
                let c2 = unchecked(c[2]);
                x_median += c0;
                z_median += c2;
                if (c0 < x_min) { x_min = c0; }
                if (c0 > x_max) { x_max = c0; }
                if (c2 < z_min) { z_min = c2; }
                if (c2 > z_max) { z_max = c2; }
            }
            let split_axis = x_max - x_min < z_max - z_min ? 2 : 0;
            let median = (split_axis == 0 ? x_median : z_median) / <f32>nodesLen;

            //reserve arrays for all nodes
            let left = new StaticArray<NavmeshNode>(nodesLen);
            let right = new StaticArray<NavmeshNode>(nodesLen);
            let left_count = 0;
            let right_count = 0;

            for (let i = 0; i < nodesLen; i++) {
                let node = nodes[i];
                let c = node.get_center();
                if (split_axis == 0) {
                    if (unchecked(c[0]) < median) {
                        unchecked(left[left_count] = node);
                        left_count++;
                    } else {
                        unchecked(right[right_count] = node);
                        right_count++;
                    }
                } else {
                    if (unchecked(c[2]) < median) {
                        unchecked(left[left_count] = node);
                        left_count++;
                    } else {
                        unchecked(right[right_count] = node);
                        right_count++;
                    }
                }
            }

            //check that both arrays left and right are non-empty
            if (left_count == 0) {
                unchecked(left[left_count] = right[right_count - 1]);
                ++left_count;
                --right_count;
            } else {
                if (right.length == 0) {
                    unchecked(right[right_count] = left[left_count - 1]);
                    ++right_count;
                    --left_count;
                }
            }

            //next create static arrays and pass it to children nodes
            let left_nodes = new StaticArray<NavmeshNode>(left_count);
            let right_nodes = new StaticArray<NavmeshNode>(right_count);
            for (let i = 0; i < left_count; i++) {
                unchecked(left_nodes[i] = left[i]);
            }
            for (let i = 0; i < right_count; i++) {
                unchecked(right_nodes[i] = right[i]);
            }

            unchecked(this.m_children[0] = new INavmeshBVH(left_nodes, BVH_AABB_DELTA));
            unchecked(this.m_children[1] = new INavmeshBVH(right_nodes, BVH_AABB_DELTA));
            this.m_children_exists = true;

            //finally, set aabb
            let left_aabb  = this.m_children[0].get_aabb();
            let right_aabb = this.m_children[1].get_aabb();

            unchecked(this.m_aabb[0] = Mathf.min(left_aabb[0], right_aabb[0]));
            unchecked(this.m_aabb[1] = Mathf.min(left_aabb[1], right_aabb[1]));
            unchecked(this.m_aabb[2] = Mathf.min(left_aabb[2], right_aabb[2]));
            unchecked(this.m_aabb[3] = Mathf.max(left_aabb[3], right_aabb[3]));
            unchecked(this.m_aabb[4] = Mathf.max(left_aabb[4], right_aabb[4]));
            unchecked(this.m_aabb[5] = Mathf.max(left_aabb[5], right_aabb[5]));
        }
    }

    @inline
    get_aabb(): StaticArray<f32> {
        return this.m_aabb;
    }

    @inline
    is_inside_aabb(x: f32, y: f32, z: f32): bool {
        let aabb = this.m_aabb;
        return aabb[0] < x && aabb[3] > x &&
               aabb[1] < y && aabb[4] > y &&
               aabb[2] < z && aabb[5] > z;
    }

    sample(x: f32, y: f32, z: f32): i32 {
        if (this.is_inside_aabb(x, y, z)) {
            if (this.m_children_exists) {  // this node does not contains object, but contains children
                //get left and right sample
                let left_sample  = this.m_children[0].sample(x, y, z);
                let right_sample = this.m_children[1].sample(x, y, z);
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
                            (x - unchecked(l_c[0])) * unchecked(l_n[0]) +
                            (y - unchecked(l_c[1])) * unchecked(l_n[1]) +
                            (z - unchecked(l_c[2])) * unchecked(l_n[2])
                        );
                        let right_node = this.m_index_to_node.get(right_sample);
                        let r_c = right_node.get_center();
                        let r_n = right_node.get_normal();
                        let r_dist = Mathf.abs(
                            (x - unchecked(r_c[0])) * unchecked(r_n[0]) +
                            (y - unchecked(r_c[1])) * unchecked(r_n[1]) +
                            (z - unchecked(r_c[2])) * unchecked(r_n[2])
                        );
                        return l_dist < r_dist ? left_sample : right_sample;
                    }
                }
            } else {  // this is the leaf-node, it contains object
                let firstNode = unchecked(this.m_nodes[0]);
                if (firstNode.is_point_inside(x, y, z)) {
                    return firstNode.get_index();
                } else {
                    return -1;
                }
            }
        }
        return -1;
    }

    to_string(): string {
        let to_return = "<bvh";
        if (this.m_is_object) {
            //to_return += " object " + this.m_object.get_index().toString() +
            to_return += " object " + this.m_nodes[0].get_index().toString() +
                         ", aabb: " + this.m_aabb.toString() +
                         ">";
        }
        else{
            to_return += " left: " + this.m_children[0].to_string() +
                         ", right: " + this.m_children[1].to_string() +
                         ", aabb: " + this.m_aabb.toString() +
                         ">";
        }
        return to_return;
    }

    toString(): string {
        return this.to_string();
    }
}

export class ITrianglesBVH {
    //coordinates of the triangle
    m_triangle_data: StaticArray<f32>;
    m_is_object: bool;  // true, if it contains the triangle

    m_aabb: StaticArray<f32>;
    m_children: StaticArray<ITrianglesBVH>;
    m_children_exists: bool;

    m_return_buffer: Float32Array;  // use this array to return values from sample command


    constructor(triangles_vertices: StaticArray<f32>, BVH_AABB_DELTA: f32 = 0.5) {
        this.m_return_buffer = new Float32Array(4);
        this.m_is_object = false;
        this.m_children = new StaticArray<ITrianglesBVH>(2);
        this.m_children_exists = false;
        this.m_aabb = new StaticArray<f32>(6);

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

            unchecked(triangle_data[0] = triangles_vertices[0]);
            unchecked(triangle_data[1] = triangles_vertices[1]);
            unchecked(triangle_data[2] = triangles_vertices[2]);
            //edge e1 = v0->v1
            unchecked(triangle_data[3] = triangles_vertices[3] - triangles_vertices[0]);
            unchecked(triangle_data[4] = triangles_vertices[4] - triangles_vertices[1]);
            unchecked(triangle_data[5] = triangles_vertices[5] - triangles_vertices[2]);
            //edge e2 = v0->v2
            unchecked(triangle_data[6] = triangles_vertices[6] - triangles_vertices[0]);
            unchecked(triangle_data[7] = triangles_vertices[7] - triangles_vertices[1]);
            unchecked(triangle_data[8] = triangles_vertices[8] - triangles_vertices[2]);
            //a = (e1, e1)
            unchecked(triangle_data[9] = this._squaredLen(
                unchecked(triangle_data[3]),
                unchecked(triangle_data[4]),
                unchecked(triangle_data[5])
            ));
            //b = (e1, e2)
            unchecked(triangle_data[10] = (
                unchecked(triangle_data[3] * triangle_data[6]) +
                unchecked(triangle_data[4] * triangle_data[7]) +
                unchecked(triangle_data[5] * triangle_data[8])
            ));
            //c = (e2, e2)
            unchecked(triangle_data[11] = this._squaredLen(
                unchecked(triangle_data[6]),
                unchecked(triangle_data[7]),
                unchecked(triangle_data[8])
            ));
            //determinant [[a b], [b c]]
            unchecked(triangle_data[12] = (
                unchecked(triangle_data[9]  * triangle_data[11]) -
                unchecked(triangle_data[10] * triangle_data[10])
            ));

            this.m_is_object = true;

            //calculate aabb of the triangle
            this.m_aabb[0] = this._min3(
                unchecked(triangles_vertices[0]),
                unchecked(triangles_vertices[3]),
                unchecked(triangles_vertices[6])
            );
            this.m_aabb[1] = this._min3(
                unchecked(triangles_vertices[1]),
                unchecked(triangles_vertices[4]),
                unchecked(triangles_vertices[7])
            );
            this.m_aabb[2] = this._min3(
                unchecked(triangles_vertices[2]),
                unchecked(triangles_vertices[5]),
                unchecked(triangles_vertices[8])
            );
            this.m_aabb[3] = this._max3(
                unchecked(triangles_vertices[0]),
                unchecked(triangles_vertices[3]),
                unchecked(triangles_vertices[6])
            );
            this.m_aabb[4] = this._max3(
                unchecked(triangles_vertices[1]),
                unchecked(triangles_vertices[4]),
                unchecked(triangles_vertices[7])
            );
            this.m_aabb[5] = this._max3(
                unchecked(triangles_vertices[2]),
                unchecked(triangles_vertices[5]),
                unchecked(triangles_vertices[8])
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
                    unchecked(triangles_vertices[9 * i + 0]),
                    unchecked(triangles_vertices[9 * i + 3]),
                    unchecked(triangles_vertices[9 * i + 6])
                );
                min_y = this._min4(
                    min_y,
                    unchecked(triangles_vertices[9 * i + 1]),
                    unchecked(triangles_vertices[9 * i + 4]),
                    unchecked(triangles_vertices[9 * i + 7])
                );
                min_z = this._min4(
                    min_z,
                    unchecked(triangles_vertices[9 * i + 2]),
                    unchecked(triangles_vertices[9 * i + 5]),
                    unchecked(triangles_vertices[9 * i + 8])
                );

                max_x = this._max4(
                    max_x,
                    unchecked(triangles_vertices[9 * i + 0]),
                    unchecked(triangles_vertices[9 * i + 3]),
                    unchecked(triangles_vertices[9 * i + 6])
                );
                max_y = this._max4(
                    max_y,
                    unchecked(triangles_vertices[9 * i + 1]),
                    unchecked(triangles_vertices[9 * i + 4]),
                    unchecked(triangles_vertices[9 * i + 7])
                );
                max_z = this._max4(
                    max_z,
                    unchecked(triangles_vertices[9 * i + 2]),
                    unchecked(triangles_vertices[9 * i + 5]),
                    unchecked(triangles_vertices[9 * i + 8])
                );

                median_x += unchecked(triangles_vertices[9 * i + 0]);
                median_x += unchecked(triangles_vertices[9 * i + 3]);
                median_x += unchecked(triangles_vertices[9 * i + 6]);

                median_z += unchecked(triangles_vertices[9 * i + 2]);
                median_z += unchecked(triangles_vertices[9 * i + 5]);
                median_z += unchecked(triangles_vertices[9 * i + 8]);
            }

            this.m_aabb[0] = min_x;
            this.m_aabb[1] = min_y;
            this.m_aabb[2] = min_z;
            this.m_aabb[3] = max_x;
            this.m_aabb[4] = max_y;
            this.m_aabb[5] = max_z;

            this._extend_aabb_by_delta(BVH_AABB_DELTA);

            let total: f32 = 1.0 / <f32>(objects_count * 3);
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
                    unchecked(triangles_vertices[9 * i + 0]) +
                    unchecked(triangles_vertices[9 * i + 3]) +
                    unchecked(triangles_vertices[9 * i + 6])
                ) / 3.0;

                let c_z: f32 = (
                    unchecked(triangles_vertices[9 * i + 2]) +
                    unchecked(triangles_vertices[9 * i + 5]) +
                    unchecked(triangles_vertices[9 * i + 8])
                ) / 3.0;

                if ((axis == 0 && c_x < median_x) || (axis == 2 && c_z < median_z)) { //add to the left
                    unchecked(left_objects[9 * left_count + 0] = triangles_vertices[9 * i + 0]);
                    unchecked(left_objects[9 * left_count + 1] = triangles_vertices[9 * i + 1]);
                    unchecked(left_objects[9 * left_count + 2] = triangles_vertices[9 * i + 2]);

                    unchecked(left_objects[9 * left_count + 3] = triangles_vertices[9 * i + 3]);
                    unchecked(left_objects[9 * left_count + 4] = triangles_vertices[9 * i + 4]);
                    unchecked(left_objects[9 * left_count + 5] = triangles_vertices[9 * i + 5]);

                    unchecked(left_objects[9 * left_count + 6] = triangles_vertices[9 * i + 6]);
                    unchecked(left_objects[9 * left_count + 7] = triangles_vertices[9 * i + 7]);
                    unchecked(left_objects[9 * left_count + 8] = triangles_vertices[9 * i + 8]);

                    left_count++;
                } else {//add to the right
                    unchecked(right_objects[9 * right_count + 0] = triangles_vertices[9 * i + 0]);
                    unchecked(right_objects[9 * right_count + 1] = triangles_vertices[9 * i + 1]);
                    unchecked(right_objects[9 * right_count + 2] = triangles_vertices[9 * i + 2]);

                    unchecked(right_objects[9 * right_count + 3] = triangles_vertices[9 * i + 3]);
                    unchecked(right_objects[9 * right_count + 4] = triangles_vertices[9 * i + 4]);
                    unchecked(right_objects[9 * right_count + 5] = triangles_vertices[9 * i + 5]);

                    unchecked(right_objects[9 * right_count + 6] = triangles_vertices[9 * i + 6]);
                    unchecked(right_objects[9 * right_count + 7] = triangles_vertices[9 * i + 7]);
                    unchecked(right_objects[9 * right_count + 8] = triangles_vertices[9 * i + 8]);

                    right_count++;
                }
            }

            //check non-infinite recursion
            if (left_count > 0 && right_count == 0) {  // move last left object to the right
                unchecked(right_objects[0] = left_objects[(left_count - 1) * 9 + 0]);
                unchecked(right_objects[1] = left_objects[(left_count - 1) * 9 + 1]);
                unchecked(right_objects[2] = left_objects[(left_count - 1) * 9 + 2]);

                unchecked(right_objects[3] = left_objects[(left_count - 1) * 9 + 3]);
                unchecked(right_objects[4] = left_objects[(left_count - 1) * 9 + 4]);
                unchecked(right_objects[5] = left_objects[(left_count - 1) * 9 + 5]);

                unchecked(right_objects[6] = left_objects[(left_count - 1) * 9 + 6]);
                unchecked(right_objects[7] = left_objects[(left_count - 1) * 9 + 7]);
                unchecked(right_objects[8] = left_objects[(left_count - 1) * 9 + 8]);

                right_count++;
                left_count--;
            } else if (left_count == 0 && right_count > 0) {  // move last right object to the left

                unchecked(left_objects[0] = right_objects[(right_count - 1) * 9 + 0]);
                unchecked(left_objects[1] = right_objects[(right_count - 1) * 9 + 1]);
                unchecked(left_objects[2] = right_objects[(right_count - 1) * 9 + 2]);

                unchecked(left_objects[3] = right_objects[(right_count - 1) * 9 + 3]);
                unchecked(left_objects[4] = right_objects[(right_count - 1) * 9 + 4]);
                unchecked(left_objects[5] = right_objects[(right_count - 1) * 9 + 5]);

                unchecked(left_objects[6] = right_objects[(right_count - 1) * 9 + 6]);
                unchecked(left_objects[7] = right_objects[(right_count - 1) * 9 + 7]);
                unchecked(left_objects[8] = right_objects[(right_count - 1) * 9 + 8]);

                left_count++;
                right_count--;
            }

            //finally, create two child nodes
            //create typed arrays and fill it
            var left_array  = new StaticArray<f32>(left_count * 9);
            var right_array = new StaticArray<f32>(right_count * 9);

            for (let j = 0; j < left_count * 9; j++) {
                unchecked(left_array[j] = left_objects[j]);
            }

            for (let j = 0; j < right_count * 9; j++) {
                unchecked(right_array[j] = right_objects[j]);
            }

            this.m_children[0] = new ITrianglesBVH(left_array, BVH_AABB_DELTA);
            this.m_children[1] = new ITrianglesBVH(right_array, BVH_AABB_DELTA);
            this.m_children_exists = true;
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
        return this.m_aabb[3] - this.m_aabb[0];
    }

    @inline
    _get_aabb_y_size(): f32 {
        return this.m_aabb[4] - this.m_aabb[1];
    }

    @inline
    _get_aabb_z_size(): f32 {
        return this.m_aabb[5] - this.m_aabb[1];
    }

    @inline
    _extend_aabb_by_delta(delta: f32): void {
        this.m_aabb[0] -= delta;
        this.m_aabb[1] -= delta;
        this.m_aabb[2] -= delta;

        this.m_aabb[3] += delta;
        this.m_aabb[4] += delta;
        this.m_aabb[5] += delta;
    }

    @inline
    _get_aabb(): StaticArray<f32> {
        return this.m_aabb;
    }

    @inline
    _is_inside_aabb(x: f32, y: f32, z: f32): bool {
        return (
            x > this.m_aabb[0] && x < this.m_aabb[3] &&
            y > this.m_aabb[1] && y < this.m_aabb[4] &&
            z > this.m_aabb[2] && z < this.m_aabb[5]
        );
    }

    @inline
    _clamp(a: f32, v_min: f32 = 0.0, v_max: f32 = 1.0): f32 {
        if (a < v_min) {
            return v_min;
        } else if (a > v_max) {
            return v_max
        } else {
            return a;
        }
    }

    @inline
    _squaredLen(x: f32, y: f32, z: f32): f32 {
        return x * x + y * y + z * z;
    }

    sample(x: f32, y: f32, z: f32): Float32Array {
        let triangle_data = this.m_triangle_data;
        //return the 4-th [x, y, z, w], where w = 1.0 - correct answer, 0.0 - empty answer
        if (this._is_inside_aabb(x, y, z)) {  // point inside aabb, so, check the node
            if (this.m_is_object) {  // this node contains object, so, return actual closest position in the triangle
                //here we should find actual closest point
                let v0_x = unchecked(triangle_data[0]) - x;
                let v0_y = unchecked(triangle_data[1]) - y;
                let v0_z = unchecked(triangle_data[2]) - z;

                //d = (e1, v0)
                let d = (
                    unchecked(triangle_data[3]) * v0_x +
                    unchecked(triangle_data[4]) * v0_y +
                    unchecked(triangle_data[5]) * v0_z
                );
                //e = (e2, v0)
                let e = (
                    unchecked(triangle_data[6]) * v0_x +
                    unchecked(triangle_data[7]) * v0_y +
                    unchecked(triangle_data[8]) * v0_z
                );

                //s = b*e - c*d
                //t = b*d - a*e
                let s = (
                    unchecked(triangle_data[10]) * e -
                    unchecked(triangle_data[11]) * d
                );
                let t = (
                    unchecked(triangle_data[10]) * d -
                    unchecked(triangle_data[9])  * e
                );

                //det = this.triangle_data[12]
                //a = this.triangle_data[9]
                //b = this.triangle_data[10]
                //c = this.triangle_data[11]
                if (s + t < unchecked(triangle_data[12])) {
                    if (s < 0) {
                        if (t < 0) {
                            if (d < 0) {
                                s = this._clamp(-d / unchecked(triangle_data[9]));
                                t = 0;
                            } else {
                                s = 0;
                                t = this._clamp(-e / unchecked(triangle_data[11]));
                            }
                        } else {
                            s = 0;
                            t = this._clamp(-e / unchecked(triangle_data[11]));
                        }
                    } else if (t < 0) {
                        s = this._clamp(-d / unchecked(triangle_data[9]));
                        t = 0.0;
                    } else {
                        let invDet: f32 = 1.0 / unchecked(triangle_data[12]);
                        s *= invDet;
                        t *= invDet;
                    }
                } else {
                    if (s < 0) {
                        let tmp0 = unchecked(triangle_data[10]) + d;
                        let tmp1 = unchecked(triangle_data[11]) + e;
                        if (tmp1 > tmp0) {
                            let numer = tmp1 - tmp0;
                            let denom = unchecked(triangle_data[9] - 2 * triangle_data[10] + triangle_data[11]);
                            s = this._clamp(numer / denom);
                            t = 1.0 - s;
                        } else {
                            t = this._clamp(-e / unchecked(triangle_data[11]));
                            s = 0;
                        }
                    }
                    else if(t < 0.0){
                        if(this.m_triangle_data[9] + d > this.m_triangle_data[10] + e) {
                            let numer = unchecked(triangle_data[11] + e - triangle_data[10] - d);
                            let denom = unchecked(triangle_data[9]  - 2 * triangle_data[10] + triangle_data[11]);
                            s = this._clamp(numer / denom);
                            t = 1.0 -  s;
                        }
                        else{
                            s = this._clamp(-d / unchecked(triangle_data[9]));
                            t = 0;
                        }
                    }
                    else{
                        let numer = unchecked(triangle_data[11] + e - triangle_data[10] - d);
                        let denom = unchecked(triangle_data[9]  - 2 * triangle_data[10] + triangle_data[11]);
                        s = this._clamp(numer / denom);
                        t = 1.0 - s;
                    }
                }

                let return_buffer = this.m_return_buffer;
                unchecked(return_buffer[0] = triangle_data[0] + s * triangle_data[3] + t * triangle_data[6]);
                unchecked(return_buffer[1] = triangle_data[1] + s * triangle_data[4] + t * triangle_data[7]);
                unchecked(return_buffer[2] = triangle_data[2] + s * triangle_data[5] + t * triangle_data[8]);
                unchecked(return_buffer[3] = 1.0);

                return return_buffer;
            } else {  // node contains children, check it

                let left_sample  = this.m_children[0].sample(x, y, z);
                let right_sample = this.m_children[1].sample(x, y, z);

                if (left_sample[3] < 0.5) {
                    return right_sample;
                } else {
                    if (right_sample[3] < 0.5) {
                        return left_sample;
                    } else {
                        //both left and right sample is correct, so, return the closest to the initial point
                        let d_l = this._squaredLen(
                            x - unchecked(left_sample[0]),
                            y - unchecked(left_sample[1]),
                            z - unchecked(left_sample[2])
                        );
                        let d_r = this._squaredLen(
                            x - unchecked(right_sample[0]),
                            y - unchecked(right_sample[1]),
                            z - unchecked(right_sample[2])
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

    to_string(): string {
        return "[(" +
            this._get_aabb().toString() + "): " +
            (this.m_is_object ? "tiangle<>" : "left-"  +
            this.m_children[0].to_string() + " right-" +
            this.m_children[1].to_string()) +
        "]";
    }

    toString(): string {
        return this.to_string();
    }
}
