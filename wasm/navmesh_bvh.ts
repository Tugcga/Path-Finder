import {NavmeshNode} from "./navmesh_node";

export class INavmeshBVH{
    m_aabb: StaticArray<f32>;
    m_children: StaticArray<INavmeshBVH>;
    m_children_exists: boolean;
    m_is_object: boolean;
    m_nodes: StaticArray<NavmeshNode>;  // all nodes, come to the current bvh

    constructor(nodes: StaticArray<NavmeshNode>, BVH_AABB_DELTA: f32 = 0.5) {
        this.m_aabb = new StaticArray<f32>(6);
        this.m_children = new StaticArray<INavmeshBVH>(2);
        this.m_children_exists = false;
        //this.m_object = new NavmeshNode(new StaticArray<f32>(0), -1, new StaticArray<i32>(0));
        this.m_is_object = false;
        this.m_nodes = nodes;

        let x_min: f32 = Infinity;
        let x_max: f32 = -Infinity;
        let y_min: f32 = Infinity;
        let y_max: f32 = -Infinity;
        let z_min: f32 = Infinity;
        let z_max: f32 = -Infinity;

        if(nodes.length == 1){  // only one object, so, but it to the current bvh node
            //this.m_object = nodes[0];  // reassign object link
            this.m_is_object = true;

            //get aabb
            //let vertices: StaticArray<f32> = this.m_object.get_vertex_coordinates();
            let vertices: StaticArray<f32> = this.m_nodes[0].get_vertex_coordinates();
            let verts_count: i32 = vertices.length / 3;
            for(let i: i32 = 0; i < verts_count; i++){
                if(vertices[3*i] < x_min){x_min = vertices[3*i];}
                if(vertices[3*i] > x_max){x_max = vertices[3*i];}
                if(vertices[3*i + 1] < y_min){y_min = vertices[3*i + 1];}
                if(vertices[3*i + 1] > y_max){y_max = vertices[3*i + 1];}
                if(vertices[3*i + 2] < z_min){z_min = vertices[3*i + 2];}
                if(vertices[3*i + 2] > z_max){z_max = vertices[3*i + 2];}
            }
            //set aabb
            this.m_aabb[0] = x_min - BVH_AABB_DELTA; this.m_aabb[1] = y_min - BVH_AABB_DELTA; this.m_aabb[2] = z_min - BVH_AABB_DELTA;
            this.m_aabb[3] = x_max + BVH_AABB_DELTA; this.m_aabb[4] = y_max + BVH_AABB_DELTA; this.m_aabb[5] = z_max + BVH_AABB_DELTA;
        }
        else{  // there are many objects, create left and right children
            let x_median: f32 = 0.0;
            let z_median: f32 = 0.0;
            for(let i: i32 = 0; i < nodes.length; i++){
                let node: NavmeshNode = nodes[i];
                let c: StaticArray<f32> = node.get_center();
                x_median += c[0];
                z_median += c[2];
                if(c[0] < x_min){x_min = c[0];}
                if(c[0] > x_max){x_max = c[0];}
                if(c[2] < z_min){z_min = c[2];}
                if(c[2] > z_max){z_max = c[2];}
            }
            let split_axis: i32 = 0;
            if((x_max - x_min) < (z_max - z_min)){
                split_axis = 2;
            }
            let median: f32;
            if(split_axis == 0){
                median = x_median / <f32>nodes.length;
            }
            else{
                median = z_median / <f32>nodes.length;
            }

            //reserve arrays for all nodes
            let left: StaticArray<NavmeshNode> = new StaticArray<NavmeshNode>(nodes.length);
            let left_count: i32 = 0;
            let right: StaticArray<NavmeshNode> = new StaticArray<NavmeshNode>(nodes.length);
            let right_count: i32 = 0;
            for(let i: i32 = 0; i < nodes.length; i++){
                let node: NavmeshNode = nodes[i];
                let c: StaticArray<f32> = node.get_center();
                if(split_axis == 0){
                    if(c[0] < median){
                        left[left_count] = node;
                        left_count++;
                    }
                    else{
                        right[right_count] = node;
                        right_count++;
                    }
                }
                else{
                    if(c[2] < median){
                        left[left_count] = node;
                        left_count++;
                    }
                    else{
                        right[right_count] = node;
                        right_count++;
                    }
                }
            }

            //check that both arrays left and right are non-empty
            if(left_count == 0){
                left[left_count] = right[right_count - 1];
                left_count++;
                right_count--;
            }
            else{
                if(right.length == 0){
                    right[right_count] = left[left_count - 1];
                    right_count++;
                    left_count--;
                }
            }

            //next create static arrays and pass it to children nodes
            let left_nodes: StaticArray<NavmeshNode> = new StaticArray<NavmeshNode>(left_count);
            let right_nodes: StaticArray<NavmeshNode> = new StaticArray<NavmeshNode>(right_count);
            for(let i: i32 = 0; i < left_count; i++){
                left_nodes[i] = left[i];
            }
            for(let i: i32 = 0; i < right_count; i++){
                right_nodes[i] = right[i];
            }

            this.m_children[0] = new INavmeshBVH(left_nodes, BVH_AABB_DELTA);
            this.m_children[1] = new INavmeshBVH(right_nodes, BVH_AABB_DELTA);
            this.m_children_exists = true;

            //finally, set aabb
            let left_aabb: StaticArray<f32> = this.m_children[0].get_aabb();
            let right_aabb: StaticArray<f32> = this.m_children[1].get_aabb();

            this.m_aabb[0] = <f32>Math.min(left_aabb[0], right_aabb[0]);
            this.m_aabb[1] = <f32>Math.min(left_aabb[1], right_aabb[1]);
            this.m_aabb[2] = <f32>Math.min(left_aabb[2], right_aabb[2]);
            this.m_aabb[3] = <f32>Math.max(left_aabb[3], right_aabb[3]);
            this.m_aabb[4] = <f32>Math.max(left_aabb[4], right_aabb[4]);
            this.m_aabb[5] = <f32>Math.max(left_aabb[5], right_aabb[5]);
        }
    }

    get_aabb(): StaticArray<f32>{
        return this.m_aabb;
    }

    is_inside_aabb(x: f32, y: f32, z: f32): boolean{
        return this.m_aabb[0] < x && this.m_aabb[3] > x &&
               this.m_aabb[1] < y && this.m_aabb[4] > y &&
               this.m_aabb[2] < z && this.m_aabb[5] > z;
    }

    sample(x: f32, y: f32, z: f32): i32{
        if(this.is_inside_aabb(x, y, z)){
            if(this.m_children_exists){  // this node does not contains object, but contains children
                //get left and right sample
                let left_sample: i32 = this.m_children[0].sample(x, y, z);
                let right_sample: i32 = this.m_children[1].sample(x, y, z);
                if(left_sample == -1){
                    return right_sample;
                }
                else{
                    if(right_sample == -1){
                        return left_sample;
                    }
                    else{  // both samples are non-empty
                        let l_c: StaticArray<f32> = this.m_nodes[left_sample].get_center();
                        let l_n: StaticArray<f32> = this.m_nodes[left_sample].get_normal();
                        let l_dist: f32 = <f32>Math.abs((x - l_c[0]) * l_n[0] + (y - l_c[1]) * l_n[1] + (z - l_c[2]) * l_n[2]);

                        let r_c: StaticArray<f32> = this.m_nodes[right_sample].get_center();
                        let r_n: StaticArray<f32> = this.m_nodes[right_sample].get_normal();
                        let r_dist: f32 = <f32>Math.abs((x - r_c[0]) * r_n[0] + (y - r_c[1]) * r_n[1] + (z - r_c[2]) * r_n[2]);

                        if(l_dist < r_dist){
                            return left_sample;
                        }
                        else{
                            return right_sample;
                        }
                    }
                }
            }
            else{  // this is the leaf-node, it contains object
                if(this.m_nodes[0].is_point_inside(x, y, z)){
                    return this.m_nodes[0].get_index();
                }
                else{
                    return -1;
                }
            }
        }
        else{
            return -1;
        }
    }

    to_string(): string{
        let to_return: string = "<bvh";
        if(this.m_is_object){
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
}

export class ITrianglesBVH{
    //coordinates of the triangle
    triangle_data: StaticArray<f32>;
    is_object: boolean;  // true, if it contains the triangle

    aabb: StaticArray<f32>;
    children: StaticArray<ITrianglesBVH>;
    children_exists: boolean;

    return_buffer: Float32Array;  // use this array to return values from sample command


    constructor(triangles_vertices: StaticArray<f32>, BVH_AABB_DELTA: f32 = 0.5) {
        this.return_buffer = new Float32Array(4);
        this.is_object = false;
        this.children = new StaticArray<ITrianglesBVH>(2);
        this.children_exists = false;
        this.aabb = new StaticArray<f32>(6);
        if(triangles_vertices.length == 9){  // 9 values mean that input are one triangle (with three vertices)
            this.triangle_data = new StaticArray<f32>(13);
        }
        else{
            this.triangle_data = new StaticArray<f32>(0);
        }

        if(triangles_vertices.length == 9){  // this is one triangle, then form the object inside the node
            //copy triangle data
            //https://www.gamedev.net/forums/topic/552906-closest-point-on-triangle/
            //store first point
            this.triangle_data[0] = triangles_vertices[0]; this.triangle_data[1] = triangles_vertices[1]; this.triangle_data[2] = triangles_vertices[2];
            //edge e1 = v0->v1
            this.triangle_data[3] = triangles_vertices[3] - triangles_vertices[0];
            this.triangle_data[4] = triangles_vertices[4] - triangles_vertices[1];
            this.triangle_data[5] = triangles_vertices[5] - triangles_vertices[2];
            //edge e2 = v0->v2
            this.triangle_data[6] = triangles_vertices[6] - triangles_vertices[0];
            this.triangle_data[7] = triangles_vertices[7] - triangles_vertices[1];
            this.triangle_data[8] = triangles_vertices[8] - triangles_vertices[2];
            //a = (e1, e1)
            this.triangle_data[9] = this.triangle_data[3] * this.triangle_data[3] + this.triangle_data[4] * this.triangle_data[4] + this.triangle_data[5] * this.triangle_data[5];
            //b = (e1, e2)
            this.triangle_data[10] = this.triangle_data[3] * this.triangle_data[6] + this.triangle_data[4] * this.triangle_data[7] + this.triangle_data[5] * this.triangle_data[8];
            //c = (e2, e2)
            this.triangle_data[11] = this.triangle_data[6] * this.triangle_data[6] + this.triangle_data[7] * this.triangle_data[7] + this.triangle_data[8] * this.triangle_data[8];
            //determinant [[a b], [b c]]
            this.triangle_data[12] = this.triangle_data[9] * this.triangle_data[11] - this.triangle_data[10] * this.triangle_data[10];

            this.is_object = true;

            //calculate aabb of the triangle
            this.aabb[0] = this.min(triangles_vertices[0], triangles_vertices[3], triangles_vertices[6]);
            this.aabb[1] = this.min(triangles_vertices[1], triangles_vertices[4], triangles_vertices[7]);
            this.aabb[2] = this.min(triangles_vertices[2], triangles_vertices[5], triangles_vertices[8]);
            this.aabb[3] = this.max(triangles_vertices[0], triangles_vertices[3], triangles_vertices[6]);
            this.aabb[4] = this.max(triangles_vertices[1], triangles_vertices[4], triangles_vertices[7]);
            this.aabb[5] = this.max(triangles_vertices[2], triangles_vertices[5], triangles_vertices[8]);

            this.extend_aabb_by_delta(BVH_AABB_DELTA);
        }
        else{
            var median_x: f32 = 0.0;
            var median_z: f32 = 0.0;
            let min_x: f32 = Infinity;
            let min_y: f32 = Infinity;
            let min_z: f32 = Infinity;
            let max_x: f32 = -Infinity;
            let max_y: f32 = -Infinity;
            let max_z: f32 = -Infinity;
            var objects_count: i32 = triangles_vertices.length / 9;
            for(var i: i32 = 0; i < objects_count; i++){
                min_x = this.min4(min_x, triangles_vertices[9*i], triangles_vertices[9*i + 3], triangles_vertices[9*i + 6]);
                min_y = this.min4(min_y, triangles_vertices[9*i + 1], triangles_vertices[9*i + 4], triangles_vertices[9*i + 7]);
                min_z = this.min4(min_z, triangles_vertices[9*i + 2], triangles_vertices[9*i + 5], triangles_vertices[9*i + 8]);

                max_x = this.max4(max_x, triangles_vertices[9*i], triangles_vertices[9*i + 3], triangles_vertices[9*i + 6]);
                max_y = this.max4(max_y, triangles_vertices[9*i + 1], triangles_vertices[9*i + 4], triangles_vertices[9*i + 7]);
                max_z = this.max4(max_z, triangles_vertices[9*i + 2], triangles_vertices[9*i + 5], triangles_vertices[9*i + 8]);

                median_x += triangles_vertices[9*i];
                median_x += triangles_vertices[9*i + 3];
                median_x += triangles_vertices[9*i + 6];

                median_z += triangles_vertices[9*i + 2];
                median_z += triangles_vertices[9*i + 5];
                median_z += triangles_vertices[9*i + 8];
            }

            this.aabb[0] = min_x; this.aabb[1] = min_y; this.aabb[2] = min_z;
            this.aabb[3] = max_x; this.aabb[4] = max_y; this.aabb[5] = max_z;

            this.extend_aabb_by_delta(BVH_AABB_DELTA);

            median_x /= <f32>(objects_count * 3);
            median_z /= <f32>(objects_count * 3);

            var axis: i8 = this.get_aabb_x_size() > this.get_aabb_z_size() ? 0 : 2;  // 0 - axis parallel to z, 2 - parallel to x
            //next we shold enumerate all objects and devide it into two parts - left and right
            //create two full buffers
            var left_objects: StaticArray<f32> = new StaticArray<f32>(9 * objects_count);
            var right_objects: StaticArray<f32> = new StaticArray<f32>(9 * objects_count);
            var left_count: i32 = 0;
            var right_count: i32 = 0;
            for(i = 0; i < objects_count; i++){
                //get the center of the triangle
                var c_x: f32 = (triangles_vertices[9*i] + triangles_vertices[9*i + 3] + triangles_vertices[9*i + 6]) / 3.0;
                var c_z: f32 = (triangles_vertices[9*i + 2] + triangles_vertices[9*i + 5] + triangles_vertices[9*i + 8]) / 3.0;
                if((axis == 0 && c_x < median_x) || (axis == 2 && c_z < median_z)){//add to the left
                    for(let j: i8 = 0; j < 9; j++){
                        left_objects[9*left_count + j] = triangles_vertices[9*i + j];
                    }
                    left_count++;
                }
                else{//add to the right
                    for(let j: i8 = 0; j < 9; j++){
                        right_objects[9*right_count + j] = triangles_vertices[9*i + j];
                    }
                    right_count++;
                }
            }

            //check non-infinite recursion
            if(left_count > 0 && right_count == 0){  // move last left object to the right
                for(let j: i8 = 0; j < 9; j++){
                    right_objects[j] = left_objects[(left_count - 1) * 9 + j]
                }
                right_count++;
                left_count--;
            }
            else if(left_count ==0 && right_count > 0){  // move last right object to the left
                for(let j: i8 = 0; j < 9; j++){
                    left_objects[j] = right_objects[(right_count - 1) * 9 + j];
                }
                left_count++;
                right_count--;
            }

            //finally, create two child nodes
            //create typed arrays and fill it
            var left_array: StaticArray<f32> = new StaticArray(left_count * 9);
            var right_array: StaticArray<f32> = new StaticArray(right_count * 9);

            for(let j: i32 = 0; j < left_count * 9; j++){
                left_array[j] = left_objects[j];
            }

            for(let j: i32 = 0; j < right_count * 9; j++){
                right_array[j] = right_objects[j];
            }

            this.children[0] = new ITrianglesBVH(left_array, BVH_AABB_DELTA);
            this.children[1] = new ITrianglesBVH(right_array, BVH_AABB_DELTA);
            this.children_exists = true;
        }
    }

    min(a: f32, b: f32, c: f32): f32{
        let t: f32 = <f32>Math.min(a, b);
        return <f32>Math.min(t, c);
    }

    max(a: f32, b: f32, c: f32): f32{
        let t: f32 = <f32>Math.max(a, b);
        return <f32>Math.max(t, c);
    }

    min4(a: f32, b: f32, c: f32, d: f32): f32{
        let u: f32 = <f32>Math.min(a, b);
        let v: f32 = <f32>Math.min(c, d);
        return <f32>Math.min(u, v);
    }

    max4(a: f32, b: f32, c: f32, d: f32): f32{
        let u: f32 = <f32>Math.max(a, b);
        let v: f32 = <f32>Math.max(c, d);
        return <f32>Math.max(u, v);
    }

    get_aabb_x_size(): f32{
        return this.aabb[3] - this.aabb[0];
    }

    get_aabb_y_size(): f32{
     return this.aabb[4] - this.aabb[1];
    }

    get_aabb_z_size(): f32{
     return this.aabb[5] - this.aabb[1];
    }

    extend_aabb_by_delta(delta: f32): void{
        this.aabb[0] -= delta;
        this.aabb[1] -= delta;
        this.aabb[2] -= delta;
        this.aabb[3] += delta;
        this.aabb[4] += delta;
        this.aabb[5] += delta;
    }

    get_aabb(): StaticArray<f32>{
        return this.aabb;
    }

    is_inside_aabb(x: f32, y: f32, z: f32): boolean {
        return x > this.aabb[0] && x < this.aabb[3] && y > this.aabb[1] && y < this.aabb[4] && z > this.aabb[2] && z < this.aabb[5];
    }

    clamp(a: f32, v_min: f32, v_max: f32): f32{
        if(a < v_min){
            return v_min;
        }
        else if(a > v_max){
            return v_max
        }
        else{
            return a;
        }
    }

    sample(x: f32, y: f32, z: f32): Float32Array{
        //return the 4-th [x, y, z, w], where w = 1.0 - correct answer, 0.0 - empty answer
        if(this.is_inside_aabb(x, y, z)){  // point inside aabb, so, check the node
            if(this.is_object){  // this node contains object, so, return actual closest position in the triangle
                //here we should find actual closest point
                let v0_x: f32 = this.triangle_data[0] - x;
                let v0_y: f32 = this.triangle_data[1] - y;
                let v0_z: f32 = this.triangle_data[2] - z;

                //d = (e1, v0)
                let d: f32 = this.triangle_data[3] * v0_x + this.triangle_data[4] * v0_y + this.triangle_data[5] * v0_z;
                //e = (e2, v0)
                let e: f32 = this.triangle_data[6] * v0_x + this.triangle_data[7] * v0_y + this.triangle_data[8] * v0_z;

                //s = b*e - c*d
                //t = b*d - a*e
                let s: f32 = this.triangle_data[10]*e - this.triangle_data[11]*d;
                let t: f32 = this.triangle_data[10]*d - this.triangle_data[9]*e;
                //det = this.triangle_data[12]
                //a = this.triangle_data[9]
                //b = this.triangle_data[10]
                //c = this.triangle_data[11]
                if(s + t < this.triangle_data[12]){
                    if(s < 0.0){
                        if(t < 0.0){
                            if(d < 0.0){
                                s = this.clamp(-d/this.triangle_data[9], 0.0, 1.0);
                                t = 0.0;
                            }
                            else{
                                s = 0.0;
                                t = this.clamp(-e/this.triangle_data[11], 0.0, 1.0);
                            }
                        }
                        else{
                            s = 0.0;
                            t = this.clamp(-e/this.triangle_data[11], 0.0, 1.0);
                        }
                    }
                    else if(t < 0.0){
                        s = this.clamp(-d/this.triangle_data[9], 0.0, 1.0);
                        t = 0.0;
                    }
                    else{
                        let invDet: f32 = 1.0 / this.triangle_data[12];
                        s *= invDet;
                        t *= invDet;
                    }
                }
                else{
                    if(s < 0.0){
                        let tmp0: f32 = this.triangle_data[10] + d;
                        let tmp1: f32 = this.triangle_data[11] + e;
                        if(tmp1 > tmp0){
                            let numer: f32 = tmp1 - tmp0;
                            let denom: f32 = this.triangle_data[9]-2*this.triangle_data[10]+this.triangle_data[11];
                            s = this.clamp(numer/denom, 0.0, 1.0);
                            t = 1-s;
                        }
                        else{
                            t = this.clamp(-e/this.triangle_data[11], 0.0, 1.0);
                            s = 0.0;
                        }
                    }
                    else if(t < 0.0){
                        if(this.triangle_data[9] + d > this.triangle_data[10] + e){
                            let numer: f32 = this.triangle_data[11] + e - this.triangle_data[10] - d;
                            let denom: f32 = this.triangle_data[9] - 2*this.triangle_data[10] + this.triangle_data[11];
                            s = this.clamp(numer/denom, 0.0, 1.0);
                            t = 1-s;
                        }
                        else{
                            s = this.clamp(-e/this.triangle_data[11], 0.0, 1.0);
                            t = 0.0;
                        }
                    }
                    else{
                        let numer: f32 = this.triangle_data[11] + e - this.triangle_data[10] - d;
                        let denom: f32 = this.triangle_data[9] - 2*this.triangle_data[10] + this.triangle_data[11];
                        s = this.clamp(numer/denom, 0.0, 1.0);
                        t = 1.0 - s;
                    }
                }
                this.return_buffer[0] = this.triangle_data[0] + s * this.triangle_data[3] + t * this.triangle_data[6];
                this.return_buffer[1] = this.triangle_data[1] + s * this.triangle_data[4] + t * this.triangle_data[7];
                this.return_buffer[2] = this.triangle_data[2] + s * this.triangle_data[5] + t * this.triangle_data[8];
                this.return_buffer[3] = 1.0;
                return this.return_buffer;
            }
            else{  // node contains children, chek it
                let left_sample: Float32Array = this.children[0].sample(x, y, z);
                let right_sample: Float32Array = this.children[1].sample(x, y, z);
                if(left_sample[3] < 0.5){
                    return right_sample;
                }
                else{
                    if(right_sample[3] < 0.5){
                        return left_sample;
                    }
                    else{
                        //both left and right sample is correct, so, return the closest to the initial point
                        let d_l: f32 = (x - left_sample[0]) * (x - left_sample[0]) + (y - left_sample[1]) * (y - left_sample[1]) + (z - left_sample[2]) * (z - left_sample[2]);
                        let d_r: f32 = (x - right_sample[0]) * (x - right_sample[0]) + (y - right_sample[1]) * (y - right_sample[1]) + (z - right_sample[2]) * (z - right_sample[2]);
                        if(d_l < d_r){
                            return left_sample;
                        }
                        else{
                            return right_sample;
                        }
                    }
                }
            }
        }
        else{  // point outside the aabb, so, skip next traversing, return false answer
            this.return_buffer[3] = 0.0;
            return this.return_buffer;
        }
    }

    to_string(): string{
        return "[(" + this.get_aabb().toString() + "): " + (this.is_object ? "tiangle<>" : "left-" + this.children[0].to_string() + " right-" + this.children[1].to_string()) + "]";
    }
}