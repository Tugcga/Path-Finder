import { log_message, Pair } from "./utilities";
import { List } from "./list";

export abstract class Serializable {
    // convert class data to the plain arrayof bytes
    abstract to_bytes(): Uint8Array;

    // fill class fields from the data in the input bytes array
    abstract from_bytes(bytes: Uint8Array): void;

    // return the number of bytes, required to store the class
    abstract bytes_length(): u32;
}

//class identificators
export enum SD_TYPE {
    SD_TYPE_UNKNOWN,
    SD_TYPE_INT32,
    SD_TYPE_FLOAT32,
    SD_TYPE_FLOAT64,
    SD_TYPE_BOOL,
    SD_TYPE_VECTOR2,
    SD_TYPE_PAIR_INT32,
    SD_TYPE_PAIR_FLOAT32,
    SD_TYPE_PAIR_FLOAT64,
    SD_TYPE_FLOAT32ARRAY,
    SD_TYPE_LIST_INT32,
    SD_TYPE_LIST_FLOAT32,
    SD_TYPE_LIST_BOOL,
    SD_TYPE_LIST_PAIR_INT32,
    SD_TYPE_LIST_FLOAT32ARRAY,
    SD_TYPE_STATICARRAY_INT32,
    SD_TYPE_STATICARRAY_STATICARRAY_INT32,
    SD_TYPE_STATICARRAY_FLOAT32,
    SD_TYPE_STATICARRAY_BOOL,
    SD_TYPE_MAP_INT32_INT32,
    SD_TYPE_MAP_INT32_STATICARRAY_INT32,
    SD_TYPE_MAP_INT32_STATICARRAY_FLOAT32,
    SD_TYPE_GRAPH,
    SD_TYPE_STATICARRAY_GRAPH,
    SD_TYPE_NAVMESHNODE,
    SD_TYPE_STATICARRAY_NAVMESHNODE,
    SD_TYPE_AABB,
    SD_TYPE_NAVMESHBVH,
    SD_TYPE_TRIANGLESBVH,
    SD_TYPE_NAVMESH
}

export function get_type(buffer: Uint8Array): i32 {
    if(buffer.length > 0){
        let view = new DataView(buffer.buffer);
        return view.getInt32(0);
    }
    else{
        return SD_TYPE.SD_TYPE_UNKNOWN;
    }
}

//-------------Bytes Length-------------
//--------------------------------------

// Float32Array
@inline
export function float32array_bytes_length(array: Float32Array): u32 {
    return 4  // id
         + 4  // bytes length
         + 4  // elements count
         + 4 * array.length;  // data
}

// List<Float32Array>
export function list_float32array_bytes_length(array: List<Float32Array>): u32 {
    //the signature: id + elements + array_0 + ... + array_{n-1}
    let size = 0;
    for(let i = 0, len = array.length; i < len; i++){
        let a = unchecked(array[i]);
        size += float32array_bytes_length(a);
    }

    return 4  // id
         + 4 // bytes length
         + 4  // elements count in the list
         + size;  // actual data length
}

// StaticArray<i32>
@inline
export function staticarray_i32_bytes_length(array: StaticArray<i32>): u32 {
    return 4 + 4 + 4 + 4 * array.length;  // id + bytes length + elements + data (4 bytes per element)
}

// StaticArray<f32>
@inline
export function staticarray_f32_bytes_length(array: StaticArray<f32>): u32 {
    return 4 + 4 + 4 + 4 * array.length;
}

// StaticArray<bool>
@inline
export function staticarray_bool_bytes_length(array: StaticArray<bool>): u32 {
    return 4 + 4 + 4 + array.length;  // one byte per element
}

// Map<i32, i32>
@inline
export function map_i32_i32_bytes_length(map: Map<i32, i32>): u32 {
    return 4 + 4 + 4 + 8 * map.size;  // id, bytes length, pairs count, 8 bytes for each pair (4 + 4)
}

// Map<i32, StaticArray<i32>>
export function map_i32_staticarray_i32_bytes_length(map: Map<i32, StaticArray<i32>>): u32 {
    let to_return = 12; // id, bytes length and elements count
    let values = map.values();
    for(let i = 0, len = values.length; i < len; i++) {
        let array = unchecked(values[i]);
        to_return += 4 + staticarray_i32_bytes_length(array);  // 4 for the key, other for the array
    }

    return to_return;
}

// Map<i32, StaticArray<f32>>
export function map_i32_staticarray_f32_bytes_length(map: Map<i32, StaticArray<f32>>): u32 {
    let to_return = 12; // id, bytes length and elements count
    let values = map.values();
    for(let i = 0, len = values.length; i < len; i++) {
        let array = unchecked(values[i]);
        to_return += 4 + staticarray_f32_bytes_length(array);  // 4 for the key, other for the array
    }

    return to_return;
}

// StaticArray<StaticArray<i32>>
export function staticarray_staticarray_i32_bytes_length(array: StaticArray<StaticArray<i32>>): u32 {
    let to_return = 12;  // id, bytes length, count
    for(let i = 0, len = array.length; i < len; i++) {
        let a = unchecked(array[i]);
        to_return += staticarray_i32_bytes_length(a);
    }
    return to_return;
}

// i32
@inline
export function i32_bytes_length(): i32 {
    return 4 + 4 + 4;  // id, bytes length, value
}

// f32
@inline
export function f32_bytes_length(): i32 {
    return 4 + 4 + 4;  // id, bytes length, value
}

// f64
@inline
export function f64_bytes_length(): i32 {
    return 4 + 4 + 8;  // id, bytes length, value
}

// bool
@inline
export function bool_bytes_length(): i32 {
    return 4 + 4 + 1;  // id, bytes length, value
}

//-------------To Bytes-------------
//----------------------------------

// Float32Array
export function float32array_to_bytes(array: Float32Array): Uint8Array {
    const bytes_length = float32array_bytes_length(array);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_FLOAT32ARRAY);
    view.setInt32(4, bytes_length);
    view.setInt32(8, array.length);
    for(let i = 0, len = array.length; i < len; i++){
        view.setFloat32(12 + 4 * i, unchecked(array[i]));
    }
    return to_return;
}

// List<Pair<i32>>
export function list_pair_i32_to_bytes(array: List<Pair<i32>>): Uint8Array {
    const bytes_length = array.bytes_length();
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_LIST_PAIR_INT32);
    view.setInt32(4, bytes_length);
    const length = array.length;
    view.setInt32(8, length);
    for(let i = 0; i < length; i++) {
        let p: Pair<i32> = array[i];
        let p_bytes = p.to_bytes();
        to_return.set(p_bytes, 12 + p.bytes_length() * i);
    }

    return to_return;
}

// List<Float32Array>
export function list_float32array_to_bytes(array: List<Float32Array>): Uint8Array {
    const bytes_length = list_float32array_bytes_length(array);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    //save id
    view.setInt32(0, SD_TYPE.SD_TYPE_LIST_FLOAT32ARRAY);
    view.setInt32(4, bytes_length);
    //save the number of list elements
    view.setInt32(8, array.length);
    let shift = 12;
    for(let i = 0, len = array.length; i < len; i++){
        let a = array[i];
        let a_bytes = float32array_to_bytes(a);
        to_return.set(a_bytes, shift);
        shift += a_bytes.length;
    }

    return to_return;
}

// StaticArray<i32>
export function staticarray_i32_to_bytes(array: StaticArray<i32>): Uint8Array {
    const bytes_length = staticarray_i32_bytes_length(array);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_STATICARRAY_INT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, array.length);
    for(let i = 0, len = array.length; i < len; i++){
        view.setInt32(12 + 4*i, unchecked(array[i]));
    }
    return to_return;
}

// StaticArray<f32>
export function staticarray_f32_to_bytes(array: StaticArray<f32>): Uint8Array {
    const bytes_length = staticarray_f32_bytes_length(array);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, array.length);
    for(let i = 0, len = array.length; i < len; i++){
        view.setFloat32(12 + 4*i, unchecked(array[i]));
    }
    return to_return;
}

// StaticArray<bool>
export function staticarray_bool_to_bytes(array: StaticArray<bool>): Uint8Array {
    const bytes_length = staticarray_bool_bytes_length(array);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_STATICARRAY_BOOL);
    view.setInt32(4, bytes_length);
    view.setInt32(8, array.length);
    for(let i = 0, len = array.length; i < len; i++){
        view.setUint8(12 + i, unchecked(array[i]) ? 1 : 0);
    }
    return to_return;
}

// Map<i32, i32>
export function map_i32_i32_to_bytes(map: Map<i32, i32>): Uint8Array {
    const bytes_length = map_i32_i32_bytes_length(map);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_MAP_INT32_INT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, map.size);
    let keys = map.keys();
    let values = map.values();
    for(let i = 0, len = keys.length; i < len; i++){
        //store key and then value
        view.setInt32(12 + 8*i, unchecked(keys[i]));
        view.setInt32(12 + 8*i + 4, unchecked(values[i]));
    }
    return to_return;
}

// Map<i32, StaticArray<i32>>
export function map_i32_staticarray_i32_to_bytes(map: Map<i32, StaticArray<i32>>): Uint8Array {
    const bytes_length = map_i32_staticarray_i32_bytes_length(map);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_MAP_INT32_STATICARRAY_INT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, map.size);
    let keys = map.keys();
    let values = map.values();
    let shift = 12;
    for(let i = 0, len = keys.length; i < len; i++){
        //at first write the key
        view.setInt32(shift, unchecked(keys[i]));
        //next write the whole array bytes
        let array = unchecked(values[i]);
        let a_bytes = staticarray_i32_to_bytes(array);
        to_return.set(a_bytes, shift + 4);

        shift += 4 + staticarray_i32_bytes_length(array);
    }
    return to_return;
}

// Map<i32, StaticArray<f32>>
export function map_i32_staticarray_f32_to_bytes(map: Map<i32, StaticArray<f32>>): Uint8Array {
    const bytes_length = map_i32_staticarray_f32_bytes_length(map);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_MAP_INT32_STATICARRAY_FLOAT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, map.size);
    let keys = map.keys();
    let values = map.values();
    let shift = 12;
    for(let i = 0, len = keys.length; i < len; i++){
        //at first write the key
        view.setInt32(shift, unchecked(keys[i]));
        //next write the whole array bytes
        let array = unchecked(values[i]);
        let a_bytes = staticarray_f32_to_bytes(array);
        to_return.set(a_bytes, shift + 4);

        shift += 4 + staticarray_f32_bytes_length(array);
    }
    return to_return;
}

// StaticArray<StaticArray<i32>>
export function staticarray_staticarray_i32_to_bytes(array: StaticArray<StaticArray<i32>>): Uint8Array {
    const bytes_length = staticarray_staticarray_i32_bytes_length(array);
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_STATICARRAY_STATICARRAY_INT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, array.length);
    let shift = 12;
    for(let i = 0, len = array.length; i < len; i++){
        let a = unchecked(array[i]);
        let a_bytes = staticarray_i32_to_bytes(a);
        const a_bytes_length = staticarray_i32_bytes_length(a);
        to_return.set(a_bytes, shift);
        shift += a_bytes_length;
    }
    return to_return;
}

// i32
export function i32_to_bytes(value: i32): Uint8Array {
    const bytes_length = i32_bytes_length();
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_INT32);
    view.setInt32(4, bytes_length);
    view.setInt32(8, value);
    return to_return;
}

// f32
export function f32_to_bytes(value: f32): Uint8Array {
    const bytes_length = f32_bytes_length();
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_FLOAT32);
    view.setInt32(4, bytes_length);
    view.setFloat32(8, value);
    return to_return;
}

// f64
export function f64_to_bytes(value: f64): Uint8Array {
    const bytes_length = f64_bytes_length();
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_FLOAT64);
    view.setInt32(4, bytes_length);
    view.setFloat64(8, value);
    return to_return;
}

// bool
export function bool_to_bytes(value: bool): Uint8Array {
    const bytes_length = bool_bytes_length();
    let to_return = new Uint8Array(bytes_length);
    let view = new DataView(to_return.buffer);
    view.setInt32(0, SD_TYPE.SD_TYPE_BOOL);
    view.setInt32(4, bytes_length);
    view.setUint8(8, value ? 1 : 0);
    return to_return;
}

//-------------From Bytes-------------
//------------------------------------

// Float32Array
export function float32array_from_bytes(bytes: Uint8Array): Float32Array {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        const id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_FLOAT32ARRAY) {
            //read the size of the array
            const count = view.getInt32(8);
            let to_return = new Float32Array(count);
            //read elements
            for(let i = 0; i < count; i++) {
                let v = view.getFloat32(12 + 4*i);
                unchecked(to_return[i] = v);
            }

            return to_return;
        }
        else {
            return new Float32Array(0);
        }
    }
    else {
        return new Float32Array(0);
    }
}

// List<Pair<i32>>
export function list_pair_i32_from_bytes(bytes: Uint8Array): List<Pair<i32>> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        const id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_LIST_PAIR_INT32) {
            const count = view.getInt32(8);
            let new_array = new List<Pair<i32>>(count);
            for(let i = 0; i < count; i++){
                let p = new Pair<i32>(0, 0);
                const p_length = p.bytes_length();
                p.from_bytes(bytes.slice(12 + p_length*i, 12 + p_length*(i + 1)));
                new_array.push(p);
            }
            return new_array;
        }
        else {
            return new List<Pair<i32>>();
        }
    }
    else {
        return new List<Pair<i32>>();
    }
}

// List<Float32Array>
export function list_float32array_from_bytes(bytes: Uint8Array): List<Float32Array> {
    if(bytes.length > 0){
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_LIST_FLOAT32ARRAY) {
            //get the number of list elements
            let count = view.getInt32(8);
            let to_return = new List<Float32Array>(count);
            let shift = 12;
            for(let i = 0; i < count; i++){
                //for each array read id and elements count
                //we need elements count to define the end of this array bytes
                let a_id = view.getInt32(shift);
                let a_size = view.getInt32(shift + 8);
                if(a_id == SD_TYPE.SD_TYPE_FLOAT32ARRAY) {
                    //convert bytes to array
                    let a = float32array_from_bytes(bytes.slice(shift, shift + 12 + a_size * 4));
                    //add it to the list
                    to_return.push(a);
                }
                shift = shift + 12 + a_size * 4;
            }
            return to_return;
        }
        else {
            return new List<Float32Array>();
        }
    }
    else {
        return new List<Float32Array>();
    }
}

// StaticArray<i32>
export function staticarray_i32_from_bytes(bytes: Uint8Array): StaticArray<i32> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
            let count = view.getInt32(8);
            let to_return = new StaticArray<i32>(count);
            for(let i = 0; i < count; i++) {
                unchecked(to_return[i] = view.getInt32(12 + 4*i));
            }
            return to_return;
        }
        else {
            return new StaticArray<i32>(0);
        }
    }
    else {
        return new StaticArray<i32>(0);
    }
}

export function staticarray_i32_from_bytes_expr(view: DataView, start: u32): StaticArray<i32> {
    const count = view.getInt32(start + 8);
    let to_return = new StaticArray<i32>(count);
    for(let i = 0; i < count; i++) {
        unchecked(to_return[i] = view.getInt32(start + 12 + 4*i));
    }
    return to_return;
}

// StaticArray<f32>
export function staticarray_f32_from_bytes(bytes: Uint8Array): StaticArray<f32> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32( 0);
        if(id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            let count = view.getInt32(8);
            let to_return = new StaticArray<f32>(count);
            for(let i = 0; i < count; i++) {
                unchecked(to_return[i] = view.getFloat32(12 + 4*i));
            }
            return to_return;
        }
        else {
            return new StaticArray<f32>(0);
        }
    }
    else {
        return new StaticArray<f32>(0);
    }
}

export function staticarray_f32_from_bytes_expr(view: DataView, start: u32): StaticArray<f32> {
    const count = view.getInt32(start + 8);
    let to_return = new StaticArray<f32>(count);
    for(let i = 0; i < count; i++) {
        unchecked(to_return[i] = view.getFloat32(start + 12 + 4*i));
    }
    return to_return;
}

// StaticArray<bool>
export function staticarray_bool_from_bytes(bytes: Uint8Array): StaticArray<bool> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_STATICARRAY_BOOL) {
            let count = view.getInt32(8);
            let to_return = new StaticArray<bool>(count);
            for(let i = 0; i < count; i++) {
                unchecked(to_return[i] = view.getUint8(12 + i) == 1);
            }
            return to_return;
        }
        else {
            return new StaticArray<bool>(0);
        }
    }
    else {
        return new StaticArray<bool>(0);
    }
}

// Map<i32, i32>
export function map_i32_i32_from_bytes(bytes: Uint8Array): Map<i32, i32> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_MAP_INT32_INT32) {
            let count = view.getInt32(8);
            let to_return = new Map<i32, i32>();
            for(let i = 0; i < count; i++) {
                to_return.set(view.getInt32(12 + 8*i), view.getInt32(12 + 8*i + 4));
            }
            return to_return;
        }
        else {
            return new Map<i32, i32>();
        }
    }
    else {
        return new Map<i32, i32>();
    }
}

export function map_i32_i32_from_bytes_expr(view: DataView, start: u32): Map<i32, i32> {
    let count = view.getInt32(start + 8);
    let to_return = new Map<i32, i32>();
    for(let i = 0; i < count; i++) {
        to_return.set(view.getInt32(start + 12 + 8*i), view.getInt32(start + 12 + 8*i + 4));
    }
    return to_return;
}

// Map<i32, StaticArray<i32>>
export function map_i32_staticarray_i32_from_bytes(bytes: Uint8Array): Map<i32, StaticArray<i32>> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_MAP_INT32_STATICARRAY_INT32) {
            let count = view.getInt32(8);
            let to_return = new Map<i32, StaticArray<i32>>();
            let shift = 12;
            for(let i = 0; i < count; i++) {
                const k = view.getInt32(shift);
                const v_id = view.getInt32(shift + 4);
                const v_bytes = view.getInt32(shift + 8);
                const v_size = view.getInt32(shift + 12);
                if(v_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
                    let array = staticarray_i32_from_bytes(bytes.slice(shift + 4, shift + 4 + v_bytes));  // + 4 is for key

                    to_return.set(k, array);
                }
                shift += 4 + v_bytes;
            }
            return to_return;
        }
        else {
            return new Map<i32, StaticArray<i32>>();
        }
    }
    else {
        return new Map<i32, StaticArray<i32>>();
    }
}

export function map_i32_staticarray_i32_from_bytes_expr(view: DataView, start: u32): Map<i32, StaticArray<i32>> {
    const count = view.getInt32(start + 8);
    let to_return = new Map<i32, StaticArray<i32>>();
    let shift = start + 12;
    for(let i = 0; i < count; i++) {
        const k = view.getInt32(shift);
        const v_id = view.getInt32(shift + 4);
        const v_bytes = view.getInt32(shift + 8);
        const v_size = view.getInt32(shift + 12);
        if(v_id == SD_TYPE.SD_TYPE_STATICARRAY_INT32) {
            let array = staticarray_i32_from_bytes_expr(view, shift + 4);  // + 4 is for key

            to_return.set(k, array);
        }
        shift += 4 + v_bytes;
    }
    return to_return;
}

// Map<i32, StaticArray<f32>>
export function map_i32_staticarray_f32_from_bytes(bytes: Uint8Array): Map<i32, StaticArray<f32>> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_MAP_INT32_STATICARRAY_FLOAT32) {
            let count = view.getInt32(8);
            let to_return = new Map<i32, StaticArray<f32>>();
            let shift = 12;
            for(let i = 0; i < count; i++) {
                const k = view.getInt32(shift);
                const v_id = view.getInt32(shift + 4);
                const v_bytes = view.getInt32(shift + 8);
                const v_size = view.getInt32(shift + 12);
                if(v_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
                    let array = staticarray_f32_from_bytes(bytes.slice(shift + 4, shift + 4 + v_bytes));  // + 4 is for key

                    to_return.set(k, array);
                }
                shift += 4 + v_bytes;
            }
            return to_return;
        }
        else {
            return new Map<i32, StaticArray<f32>>();
        }
    }
    else {
        return new Map<i32, StaticArray<f32>>();
    }
}

export function map_i32_staticarray_f32_from_bytes_expr(view: DataView, start: u32): Map<i32, StaticArray<f32>> {
    let count = view.getInt32(start + 8);
    let to_return = new Map<i32, StaticArray<f32>>();
    let shift = start + 12;
    for(let i = 0; i < count; i++) {
        const k = view.getInt32(shift);
        const v_id = view.getInt32(shift + 4);
        const v_bytes = view.getInt32(shift + 8);
        const v_size = view.getInt32(shift + 12);
        if(v_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            //let array = staticarray_f32_from_bytes(bytes.slice(shift + 4, shift + 4 + v_bytes));  // + 4 is for key
            let array = staticarray_f32_from_bytes_expr(view, shift + 4);
            to_return.set(k, array);
        }
        shift += 4 + v_bytes;
    }
    return to_return;
}

// StaticArray<StaticArray<i32>>
export function staticarray_staticarray_i32_from_bytes(bytes: Uint8Array): StaticArray<StaticArray<i32>> {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_STATICARRAY_STATICARRAY_INT32) {
            let count = view.getInt32(8);
            let to_return = new StaticArray<StaticArray<i32>>(count);
            let shift = 12;
            for(let i = 0; i < count; i++) {
                const a_bytes_length = view.getInt32(shift + 4);
                let a = staticarray_i32_from_bytes(bytes.slice(shift, shift + a_bytes_length));
                unchecked(to_return[i] = a);
                shift += a_bytes_length;
            }
            return to_return;
        }
        else {
            return new StaticArray<StaticArray<i32>>(0);
        }
    }
    else {
        return new StaticArray<StaticArray<i32>>(0);
    }
}

export function staticarray_staticarray_i32_from_bytes_expr(view: DataView, start: u32): StaticArray<StaticArray<i32>> {
    const count = view.getInt32(start + 8);
    let to_return = new StaticArray<StaticArray<i32>>(count);
    let shift = start + 12;
    for(let i = 0; i < count; i++) {
        const a_bytes_length = view.getInt32(shift + 4);
        let a = staticarray_i32_from_bytes_expr(view, shift);
        unchecked(to_return[i] = a);
        shift += a_bytes_length;
    }
    return to_return;
}

// i32
export function i32_from_bytes(bytes: Uint8Array): i32 {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_INT32) {
            return view.getInt32(8);
        }
        else {
            return 0;
        }
    }
    else {
        return 0;
    }
}

// f32
export function f32_from_bytes(bytes: Uint8Array): f32 {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_FLOAT32) {
            return view.getFloat32(8);
        }
        else {
            return 0.0;
        }
    }
    else {
        return 0.0;
    }
}

// f64
export function f64_from_bytes(bytes: Uint8Array): f64 {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_FLOAT64) {
            return view.getFloat64(8);
        }
        else {
            return 0.0;
        }
    }
    else {
        return 0.0;
    }
}

// bool
export function bool_from_bytes(bytes: Uint8Array): bool {
    if(bytes.length > 0) {
        let view = new DataView(bytes.buffer);
        let id = view.getInt32(0);
        if(id == SD_TYPE.SD_TYPE_BOOL) {
            return view.getUint8(8) == 1;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}