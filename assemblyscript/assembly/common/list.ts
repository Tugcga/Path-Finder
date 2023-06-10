import { Serializable, SD_TYPE } from "./binary_io";
import { log_message } from "./utilities";
import { Pair } from "./utilities";

export class List<T> extends Serializable {
    m_array: StaticArray<T>;
    m_size: i32;
    m_max_size: i32;

    constructor(start_size: i32 = 0){
        super();
        let max_size = start_size + 4;
        this.m_max_size = max_size;
        this.m_array = new StaticArray<T>(max_size);
        this.m_size = 0;
    }

    static from_array<T>(in_array: Array<T>): List<T> {
        var to_return = new List<T>(in_array.length);
        for(let i = 0, len = in_array.length; i < len; i++) {
            to_return.push(in_array[i]);
        }
        return to_return;
    }

    @inline
    reset(): void{
        this.m_size = 0;
    }

    @inline
    copy_from(in_list: List<T>): void {
        const in_size = in_list.length;
        if(in_size > this.m_max_size) {
            this.m_max_size = in_size;
            this.m_array = new StaticArray<T>(in_size);
        }

        this.m_size = in_size;
        for(let i = 0; i < in_size; i++) {
            this.m_array[i] = in_list.get(i);
        }
    }

    @inline
    extend(other: List<T>): void {
        const current_size = this.m_size;
        const other_size = other.length;
        const total_size = current_size + other_size;
        if(total_size > this.m_max_size) {
            this.m_max_size = total_size;
            const new_array = new StaticArray<T>(total_size);
            for(let i = 0; i < current_size; i++) {
                new_array[i] = this.m_array[i];
            }

            this.m_array = new_array;
        }

        for(let i = 0; i < other_size; i++) {
            this.m_array[current_size + i] = other.get(i);
        }
        this.m_size = total_size;
    }

    @inline
    push(value: T): void {
        let size = this.m_size;
        let max_size = this.m_max_size;

        if(size == max_size){
            max_size += max_size / 4 + 4;
            let new_array = new StaticArray<T>(max_size);
            memory.copy(
                changetype<usize>(new_array),
                changetype<usize>(this.m_array),
                size * sizeof<T>()
            );
            this.m_array = new_array;
            this.m_max_size = max_size;
        }

        this.m_array[size] = value;
        this.m_size = size + 1;
    }

    @inline
    pop(index: i32): T{
        let arr = this.m_array;
        let len = this.m_size - 1;
        let to_return = arr[index];
        // shift all other values
        for(let i = index; i < len; ++i){
            arr[i] = arr[i + 1];
        }
        this.m_size = len;
        return to_return;
    }

    @inline
    pop_value(value: T): T {
        // find value in the array
        for(let i = 0, len = this.m_size; i < len; i++) {
            if(this.m_array[i] == value) {
                return this.pop(i);
            }
        }

        return value;
    }

    @inline
    pop_sequence(index: i32, length: i32): void{
        // remove elements from index, index + 1, ..., index + length - 1
        let arr = this.m_array;
        for(let i = index, len = this.m_size - length; i < len; ++i){
            arr[i] = arr[i + length];
        }
        this.m_size -= length;
    }

    @inline
    pop_last(): T{
        let size = this.m_size;
        this.m_size = size - 1;
        return this.m_array[size - 1];
    }

    @inline
    sort(): void{
        this.m_array = this.m_array.slice<StaticArray<T>>(0, this.m_size).sort();
        this.m_size = this.m_array.length;
        this.m_max_size = this.m_size;
    }

    @inline
    get length(): i32{
        return this.m_size;
    }

    @inline
    get(index: i32): T {
        return this.m_array[index];
    }

    @inline
    set(index: i32, value: T): void {
        this.m_array[index] = value;
    }

    @inline
    @operator("[]") private __get(index: i32): T {
        return this.m_array[index];
    }

    @inline
    @operator("[]=") private __set(index: i32, value: T): void {
        this.m_array[index] = value;
    }

    //use these methods only for primitive types (i32, f32 and bool)
    override to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        if(to_return.length > 0) {
            let view = new DataView(to_return.buffer);
            let type!: T;
            if(type instanceof i32) {
                view.setInt32(0, SD_TYPE.SD_TYPE_LIST_INT32);
                view.setInt32(4, bytes_length);
                view.setInt32(8, this.m_size);
                for(let i = 0, len = this.m_size; i < len; i++) {
                    view.setInt32(12 + 4*i, <i32>this.m_array[i]);
                }
            }
            else if(type instanceof f32) {
                view.setInt32(0, SD_TYPE.SD_TYPE_LIST_FLOAT32);
                view.setInt32(4, bytes_length);
                view.setInt32(8, this.m_size);
                for(let i = 0, len = this.m_size; i < len; i++) {
                    view.setFloat32(12 + 4*i, <f32>this.m_array[i]);
                }
            }
            else if(type instanceof bool) {
                view.setInt32(0, SD_TYPE.SD_TYPE_LIST_BOOL);
                view.setInt32(4, bytes_length);
                view.setInt32(8, this.m_size);
                for(let i = 0, len = this.m_size; i < len; i++) {
                    view.setUint8(12 + i, <bool>this.m_array[i] ? 1 : 0);
                }
            }
        }

        return to_return;
    }

    override from_bytes(bytes: Uint8Array): void {
        if(bytes.length > 0) {
            let view = new DataView(bytes.buffer);
            const id = view.getInt32(0);
            const count = view.getInt32(8);
            let new_array = new StaticArray<T>(count);
            this.m_size = count;
            this.m_max_size = count;
            for(let i = 0; i < count; i++){
                if(id == SD_TYPE.SD_TYPE_LIST_INT32) {
                    new_array[i] = <T>view.getInt32(12 + 4*i);
                }
                else if(id == SD_TYPE.SD_TYPE_LIST_FLOAT32) {
                    new_array[i] = <T>view.getFloat32(12 + 4*i);
                }
                else if(id == SD_TYPE.SD_TYPE_LIST_BOOL) {
                    const v = view.getUint8(12 + i) == 1 ? true : false;
                    new_array[i] = <T>v;
                }
            }
            this.m_array = new_array;
        }
    }

    //calculate the byte length of the list for types with fixed size
    override bytes_length(): u32 {
        let type!: T;
        if(type instanceof i32) {
            return 4  // id
                 + 4  // byte length
                 + 4  // length
                 + 4 * this.m_size;  // elements
        }
        else if(type instanceof f32) {
            return 4 + 4 + 4 + 4 * this.m_size;
        }
        else if(type instanceof bool) {
            return 4 + 4 + 4 + this.m_size;  // store bool as u8 (one byte)
        }
        else if(type instanceof Pair<i32>) {
            return 4 + 4 + 4 + 16 * this.m_size;  // each i32 pair is (4 + 4 + 4 + 4)
        }
        return 0;
    }

    @inline
    to_static(): StaticArray<T>{
        return this.m_array.slice<StaticArray<T>>(0, this.m_size);
    }

    toString(): string{
        return `[${this.to_static()}]`;
    }
}
