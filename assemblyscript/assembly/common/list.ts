export class List<T>{
    m_array: StaticArray<T>;
    m_size: i32;
    m_max_size: i32;

    constructor(start_size: i32 = 0){
        let max_size = start_size + 4;
        this.m_max_size = max_size;
        this.m_array = new StaticArray<T>(max_size);
        this.m_size = 0;
    }

    @inline
    reset(): void{
        this.m_size = 0;
    }

    @inline
    push(value: T): void{
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
        }

        unchecked(this.m_array[size] = value);
        this.m_size = size + 1;
    }

    @inline
    pop(index: i32): T{
        let arr = this.m_array;
        let len = this.m_size - 1;
        let to_return = arr[index];
        // shift all other values
        for(let i = index; i < len; ++i){
            unchecked(arr[i] = arr[i + 1]);
        }
        this.m_size = len;
        return to_return;
    }

    @inline
    pop_sequence(index: i32, length: i32): void{
        // remove elements from index, index + 1, ..., index + length - 1
        let arr = this.m_array;
        for(let i = index, len = this.m_size - length; i < len; ++i){
            unchecked(arr[i] = arr[i + length]);
        }
        this.m_size -= length;
    }

    @inline
    pop_last(): T{
        let size = this.m_size;
        this.m_size = size - 1;
        return unchecked(this.m_array[size - 1]);
    }

    @inline
    sort(): void{
        this.m_array = this.m_array.slice(0, this.m_size).sort();
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
        return this.m_array.slice(0, this.m_size);
    }

    toString(): string{
        return `[${this.to_static()}]`;
    }
}
