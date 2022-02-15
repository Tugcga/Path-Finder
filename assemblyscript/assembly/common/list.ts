import { log_message } from "./utilities";

export class List<T>{
    m_array: StaticArray<T>;
    m_size: i32;
    m_max_size: i32;

    constructor(start_size: i32 = 0){
        this.m_max_size = start_size + 4;
        this.m_array = new StaticArray<T>(this.m_max_size);
        this.m_size = 0;
    }

    @inline
    reset(): void{
        this.m_size = 0;
    }

    @inline
    push(value: T): void{
        if(this.m_size == this.m_max_size){
            this.m_max_size = this.m_max_size + this.m_max_size / 4 + 4;
            var new_array = new StaticArray<T>(this.m_max_size);
            for(let i = 0; i < this.m_size; i++){
                unchecked(new_array[i] = this.m_array[i]);
            }
            this.m_array = new_array;
        }

        unchecked(this.m_array[this.m_size] = value);
        this.m_size++;
    }

    @inline
    pop(index: i32): T{
        var to_return = this.m_array[index];
        // shift all other values
        for(let i = index; i < this.m_size - 1; i++){
            unchecked(this.m_array[i] = this.m_array[i + 1]);
        }
        this.m_size--;
        return to_return;
    }

    @inline
    pop_sequence(index: i32, length: i32): void{
        // remove elements from index, index + 1, ..., index + length - 1
        for(let i = index; i < this.m_size - length; i++){
            unchecked(this.m_array[i] = this.m_array[i + length]);
        }
        this.m_size = this.m_size - length;
    }

    @inline
    pop_last(): T{
        this.m_size--;
        return unchecked(this.m_array[this.m_size]);
    }

    @inline
    sort(): void{
        let new_array = this.m_array.slice(0, this.m_size);
        new_array.sort();
        for(let i = 0, len = this.m_size; i < len; i++){
            unchecked(this.m_array[i] = new_array[i]);
        }
    }

    @inline
    get length(): i32{
        return this.m_size;
    }

    @inline
    @operator("[]") private __get(index: i32): T {
        return unchecked(this.m_array[index]);
    }

    @inline
    @operator("[]=") private __set(index: i32, value: T): void {
        unchecked(this.m_array[index] = value);
    }

    @inline
    to_static(): StaticArray<T>{
        var to_return = new StaticArray<T>(this.m_size);
        for(let i = 0; i < this.m_size; i++){
            to_return[i] = this.m_array[i];
        }
        return to_return;
    }

    toString(): string{
        return "[" + this.m_array.slice(0, this.m_size).toString() + "]";
    }
}