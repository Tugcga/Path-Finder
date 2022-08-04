import "wasi";
import { Vector2, abs_sq, dot } from "./vector2";
import { List } from "./list";

export function log_message(message: string): void{
    console.log(message);
}

export const RVO_EPSILON: f32 = 0.0001;
export const RVO_INFINITY: f32 = <f32>Number.MAX_VALUE;

export class Pair<T>{
    constructor(public m_x: T, public m_y: T) {}

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

    toString(): string{
        return `(${this.m_x}, ${this.m_y})`;
    }
}

@inline
export function is_edge_new(edges: Array<i32>, a: i32, b: i32): bool {
    for (let i = 0, len = edges.length / 2; i < len; i++) {
        let e0 = unchecked(edges[2 * i + 0]);
        let e1 = unchecked(edges[2 * i + 1]);
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
export function distance(a_x: f32, a_y: f32, a_z: f32, b_x: f32, b_y: f32, b_z: f32): f32 {
    let dx = a_x - b_x;
    let dy = a_y - b_y;
    let dz = a_z - b_z;
    return Mathf.sqrt(dx * dx + dy * dy + dz * dz);
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
