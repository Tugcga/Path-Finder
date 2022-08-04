import { log_message } from "./utilities";

export abstract class Serializable {
    abstract to_bytes(): Uint8Array;
    /*Signature of the data for each class should be the following:
    start from 4 bytes - class identificator
    next 4 bytes - the length of the data in bytes
    other bytes - actual data

    so, the result is length + 4 + 4 bytes

    */
    abstract from_bytes(bytes: Uint8Array): void;
}

//class identificators
export const SD_TYPE_INT32 = 0;
export const SD_TYPE_FLOAT32 = 1;
export const SD_TYPE_FLOAT64 = 2;
export const SD_TYPE_VECTOR2 = 3;

let convert_buffer = new Uint8Array(8);
let convert_view = new DataView(convert_buffer.buffer);

export function float32_to_bytes(value: f32): Uint8Array {
    convert_view.setFloat32(0, value);
    let to_return = new Uint8Array(4);
    for(let i = 0; i < 4; i++) {
        to_return[i] = convert_buffer[i];
    }
    return to_return;
}

export function int32_to_bytes(value: i32): Uint8Array {
    convert_view.setInt32(0, value);
    let to_return = new Uint8Array(4);
    for(let i = 0; i < 4; i++) {
        to_return[i] = convert_buffer[i];
    }
    return to_return;
}

export function bytes_to_i32(array: Uint8Array, position: u32): i32 {
    let data_view = new DataView(array.buffer);
    return data_view.getInt32(position);
}

export function bytes_to_f32(array: Uint8Array, position: u32): f32 {
    let data_view = new DataView(array.buffer);
    return data_view.getFloat32(position);
}

export function union_buffers(a: Uint8Array, b: Uint8Array): Uint8Array {
    let to_return = new Uint8Array(a.length + b.length);
    for(let i = 0, len = a.length; i < len; i++) {
        to_return[i] = a[i];
    }
    const l = a.length;
    for(let i = 0, len = b.length; i < len; i++) {
        to_return[l + i] = b[i];
    }
    return to_return;
}