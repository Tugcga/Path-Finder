import { Vector2, abs_sq, dot } from "./vector2";
import { List } from "./list";
import { Serializable, SD_TYPE } from "./binary_io";

export const RVO_EPSILON: f32 = 0.0001;
export const RVO_INFINITY: f32 = <f32>Number.MAX_VALUE;

export function log_message(message: string): void {
    
}

export class Pair<T> extends Serializable {
    constructor(public m_x: T, public m_y: T) {
        super();
    }

    @inline
    x(): T{
        return this.m_x;
    }

    @inline
    y(): T{
        return this.m_y;
    }

    @inline
    equal(other: Pair<T>): bool{
        if(this.m_x == other.x() && this.m_y == other.y()){
            return true;
        }
        return false;
    }

    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        if(to_return.length > 0){
            let view = new DataView(to_return.buffer);
            let type!: T;
            if(type instanceof i32){
                view.setInt32(0, SD_TYPE.SD_TYPE_PAIR_INT32);
                view.setInt32(4, bytes_length);
                view.setInt32(8, <i32>this.m_x);
                view.setInt32(12, <i32>this.m_y);
            }
            else if(type instanceof f32){
                view.setInt32(0, SD_TYPE.SD_TYPE_PAIR_FLOAT32);
                view.setInt32(4, bytes_length);
                view.setFloat32(8, <f32>this.m_x);
                view.setFloat32(12, <f32>this.m_y);
            }
            else if(type instanceof f64){
                view.setInt32(0, SD_TYPE.SD_TYPE_PAIR_FLOAT64);
                view.setInt32(4, bytes_length);
                view.setFloat64(8, <f64>this.m_x);
                view.setFloat64(16, <f64>this.m_y);
            }
            else{
                view.setInt32(0, SD_TYPE.SD_TYPE_UNKNOWN);
                view.setInt32(4, bytes_length);  // bytes_length = 0
            }
        }
        return to_return;
    }

    override from_bytes(bytes: Uint8Array): void {
        if(bytes.length > 0){
            let view = new DataView(bytes.buffer);
            const id = view.getInt32(0);
            if(id == SD_TYPE.SD_TYPE_PAIR_INT32){
                this.m_x = <T>view.getInt32(8);
                this.m_y = <T>view.getInt32(12);
            }
            else if(id == SD_TYPE.SD_TYPE_PAIR_FLOAT32){
                this.m_x = <T>view.getFloat32(8);
                this.m_y = <T>view.getFloat32(12);
            }
            else if(id == SD_TYPE.SD_TYPE_PAIR_FLOAT64){
                this.m_x = <T>view.getFloat64(8);
                this.m_y = <T>view.getFloat64(16);
            }
        }
    }

    override bytes_length(): u32 {
        let type!: T;
        if (type instanceof i32 || type instanceof f32) {
            return 4 + 4 + 4 + 4;
        }
        else if(type instanceof f64) {
            return 4 + 4 + 8 + 8;
        }
        return 0;
    }

    toString(): string {
        return `(${this.m_x}, ${this.m_y})`;
    }
}

@inline
export function is_edge_new(edges: Array<i32>, a: i32, b: i32): bool {
    for (let i = 0, len = edges.length / 2; i < len; i++) {
        let e0 = edges[2 * i + 0];
        let e1 = edges[2 * i + 1];
        if ((e0 == a && e1 == b) || (e0 == b && e1 == a)) {
            return false;
        }
    }
    return true;
}

@inline
export function squared_len(x: f32, y: f32, z: f32): f32 {
    return x * x + y * y + z * z;
}

@inline
export function distance_sq(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): f32 {
    let dx = a_x - b_x;
    let dy = a_y - b_y;
    let dz = a_z - b_z;
    return dx * dx + dy * dy + dz * dz;
}

@inline
export function distance(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): f32 {
    return Mathf.sqrt(distance_sq(a_x, a_y, a_z, b_x, b_y, b_z));
}

@inline
export function clamp(x: f32, min: f32 = 0.0, max: f32 = 1.0): f32 {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max
    } else {
        return x;
    }
}

@inline
export function left_of_points(a: Vector2, b: Vector2, c: Vector2): f32{
    let v1 = a.subtract(c);
    let v2 = b.subtract(a);
    return v1.x()*v2.y() - v1.y()*v2.x();
}

@inline
export function left_of(a_x: f32, a_y: f32,
                        b_x: f32, b_y: f32,
                        c_x: f32, c_y: f32): f32{
    const v1_x = a_x - c_x;
    const v1_y = a_y - c_y;
    const v2_x = b_x - a_x;
    const v2_y = b_y - a_y;
    return v1_x*v2_y - v1_y*v2_x;
}

@inline
export function is_in_array(value: i32, array: List<i32>): bool{
    for(let i = 0, len = array.length; i < len; i++){
        const v = array[i];
        if(value == v){
            return true;
        }
    }
    return false;
}

@inline
export function is_pair_in_list(pair: Pair<i32>, list: List<Pair<i32>>): bool{
    for(let i = 0, len = list.length; i < len; i++){
        const v = list[i];
        if(v.equal(pair)){
            return true;
        }
    }
    return false;
}

@inline
export function dist_sq_point_line_segment(a: Vector2, b: Vector2, c: Vector2): f32{
    let ac = c.subtract(a);
    let ab = b.subtract(a);

    const r = dot(ac, ab) / abs_sq(ab);

    if (r < 0.0) {
        return abs_sq(ac);
    }
    else if (r > 1.0) {
        let bc = c.subtract(b);
        return abs_sq(bc);
    }
    else {
        let rab = ab.scale(r);
        let arab = a.add(rab);
        let carab = c.subtract(arab);
        return abs_sq(carab);
    }
}

@inline
export function sqr(value: f32): f32{
    return value * value;
}

export function map_to_string(map: Map<i32, i32>): string {
    let keys = map.keys();
    let values = map.values();
    let to_return = "{";
    for(let i = 0, len = keys.length; i < len; i++){
        to_return += keys[i].toString() + ": " + values[i].toString() + (i == len - 1 ? "}" : ", ");
    }
    return to_return;
}

export function arrays_eq(a: StaticArray<f32>, b: StaticArray<f32>): bool {
    if(a.length != b.length) {
        return false;
    }

    for(let i = 0, len = a.length; i < len; i++) {
        if(Mathf.abs(a[i] - b[i]) > 0.00001) {
            return false;
        }
    }

    return true;
}

@inline
export function cross(a_x: f32, a_y: f32, a_z: f32, 
                      b_x: f32, b_y: f32, b_z: f32): StaticArray<f32> {
    return [a_y * b_z - a_z * b_y,
            a_z * b_x - a_x * b_z,
            a_x * b_y - a_y * b_x];
}

@inline
export function dot3(a_x: f32, a_y: f32, a_z: f32, 
                     b_x: f32, b_y: f32, b_z: f32): f32 {
    return a_x * b_x + a_y * b_y + a_z * b_z;
}
