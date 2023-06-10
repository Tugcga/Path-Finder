import { MBRectangle } from "./rectangle";
import { Serializable, SD_TYPE,
         staticarray_f32_bytes_length,
         staticarray_f32_to_bytes,
         staticarray_f32_from_bytes_expr } from "../../common/binary_io";

export class Polygon extends Serializable {
    private m_coordinates: StaticArray<f32>;

    constructor(in_coordinates: StaticArray<f32> = new StaticArray<f32>(0)) {
        super();
        assert(in_coordinates.length % 2 == 0, "Polygon coordinates array should be even order");

        this.m_coordinates = in_coordinates;
    }

    define_from_array(in_array: Array<f32>): void {
        assert(in_array.length % 2 == 0, "Polygon coordinates array should be even order");

        this.m_coordinates = StaticArray.fromArray<f32>(in_array);
    }

    coordinates(): StaticArray<f32> {
        return this.m_coordinates;
    }

    // return the number of corners in the polygon
    // 1 for point
    // 2 for edge
    // >= 3 for triangle and other
    corners(): u32 {
        return this.m_coordinates.length / 2;
    }

    get_containing_rectangle(insert_polygon: bool = true): MBRectangle {
        var min_x = f32.MAX_VALUE;
        var max_x = f32.MIN_VALUE;

        var min_y = f32.MAX_VALUE;
        var max_y = f32.MIN_VALUE;

        const coordinates = this.m_coordinates;
        for (let i = 0, len = coordinates.length / 2; i < len; i++) {
            const x = coordinates[2*i];
            const y = coordinates[2*i + 1];

            if (x < min_x) { min_x = x; }
            if (x > max_x) { max_x = x; }

            if (y < min_y) { min_y = y; }
            if (y > max_y) { max_y = y; }
        }

        const poly_mbr = new MBRectangle(min_x, max_y, max_x, min_y);
        if (insert_polygon) {
            poly_mbr.set_polygon(this);
        }
        return poly_mbr;
    }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_RTREE_POLYGON);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // coordinates
        let coordinates = staticarray_f32_to_bytes(this.m_coordinates);
        to_return.set(coordinates, shift);

        return to_return;
    }

    from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_RTREE_POLYGON) {
            shift += 8;
        } else { return; }

        const coordinates_id = view.getInt32(shift);
        if(coordinates_id == SD_TYPE.SD_TYPE_STATICARRAY_FLOAT32) {
            const coordinates_length = view.getInt32(shift + 4);
            this.m_coordinates = staticarray_f32_from_bytes_expr(view, shift);
            shift += coordinates_length;
        } else { return; }
    }

    bytes_length(): u32 {
        return 4 +  // id
               4 +  // length
               staticarray_f32_bytes_length(this.m_coordinates);
    }

    toString(): string {
        const coordinates = this.m_coordinates;
        const count = coordinates.length / 2;
        
        const lines_str = new StaticArray<string>(count);
        for (let i = 0; i < count; i++) {
            lines_str[i] = `(${coordinates[2*i]}, ${coordinates[2*i + 1]})`;
        }

        return "<" + lines_str.join() + ">";
    }
}

export class Point extends Polygon {
    constructor(in_x: f32 = 0.0, in_y: f32 = 0.0) {
        super(StaticArray.fromArray<f32>([in_x, in_y]));
    }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_RTREE_POINT);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // coordinates
        view.setFloat32(shift, this.m_coordinates[0]);
        shift += 4;
        view.setFloat32(shift, this.m_coordinates[1]);
        shift += 4;

        return to_return;
    }

    from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_RTREE_POINT) {
            shift += 8;
        } else { return; }

        const x = view.getFloat32(shift);
        shift += 4;
        const y = view.getFloat32(shift);
        shift += 4;

        this.m_coordinates[0] = x;
        this.m_coordinates[1] = y;
    }

    bytes_length(): u32 {
        return 4 +  // id
               4 +  // length
               4 + 4;  // f32 coordinates
    }
}

export class Edge extends Polygon {
    private m_to_x: f32;
    private m_to_y: f32;

    constructor(start_x: f32 = 0.0, start_y: f32 = 0.0, finish_x: f32 = 0.0, finish_y: f32 = 0.0) {
        super(StaticArray.fromArray<f32>([start_x, start_y, finish_x, finish_y]));

        const to_x = finish_x - start_x;
        const to_y = finish_y - start_y;

        this.m_to_x = to_x;
        this.m_to_y = to_y;
    }

    // coordinates in the edge at the time parameter t
    // the inner of the edge corresponds to the t inside (0, 1)
    x(t: f32): f32 { return this.m_coordinates[0] + (this.m_coordinates[2] - this.m_coordinates[0]) * t; }
    y(t: f32): f32 { return this.m_coordinates[1] + (this.m_coordinates[3] - this.m_coordinates[1]) * t; }

    start_x(): f32 { return this.m_coordinates[0]; }
    start_y(): f32 { return this.m_coordinates[1]; }
    finish_x(): f32 { return this.m_coordinates[2]; }
    finish_y(): f32 { return this.m_coordinates[3]; }

    to_x(): f32 { return this.m_to_x; }
    to_y(): f32 { return this.m_to_y; }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_RTREE_EDGE);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // coordinates
        view.setFloat32(shift, this.m_coordinates[0]);
        shift += 4;
        view.setFloat32(shift, this.m_coordinates[1]);
        shift += 4;
        view.setFloat32(shift, this.m_coordinates[2]);
        shift += 4;
        view.setFloat32(shift, this.m_coordinates[3]);
        shift += 4;

        // additional to_x and to_y
        view.setFloat32(shift, this.m_to_x);
        shift += 4;
        view.setFloat32(shift, this.m_to_y);
        shift += 4;

        return to_return;
    }

    from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_RTREE_EDGE) {
            shift += 8;
        } else { return; }

        const x0 = view.getFloat32(shift);
        shift += 4;
        const y0 = view.getFloat32(shift);
        shift += 4;
        const x1 = view.getFloat32(shift);
        shift += 4;
        const y1 = view.getFloat32(shift);
        shift += 4;

        const to_x = view.getFloat32(shift);
        shift += 4;
        const to_y = view.getFloat32(shift);
        shift += 4;

        this.m_coordinates[0] = x0;
        this.m_coordinates[1] = y0;
        this.m_coordinates[2] = x1;
        this.m_coordinates[3] = y1;
        this.m_to_x = to_x;
        this.m_to_y = to_y;
    }

    bytes_length(): u32 {
        return 4 +  // id
               4 +  // length
               4 + 4 + 4 + 4 +  // f32 coordinates for start and finish
               4 + 4;  // m_to_x and m_to_y
    }

    toString(): string {
        return `(${this.m_coordinates[0]}, ${this.m_coordinates[1]})-(${this.m_coordinates[2]}, ${this.m_coordinates[3]})`;
    }
}