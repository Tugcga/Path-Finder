import { Polygon, Point, Edge } from "./polygon";
import { Serializable, SD_TYPE,
         bool_bytes_length,
         bool_to_bytes } from "../../common/binary_io";

export class Rectangle extends Serializable {
    private m_left_top_x: f32;
    private m_left_top_y: f32;
    private m_right_bottom_x: f32;
    private m_right_bottom_y: f32;

    private m_length: f32;
    private m_breadth: f32;

    private m_area: f32;

    constructor(in_left_top_x: f32 = 0.0, in_left_top_y: f32 = 0.0, in_right_bottom_x: f32 = 0.0, in_right_bottom_y: f32 = 0.0) {
        super();
        assert(in_right_bottom_y <= in_left_top_y, "Top corner should have higher Y value than bottom");
        assert(in_left_top_x <= in_right_bottom_x, "Right corner should have higher X value than left");
        
        this.m_left_top_x = in_left_top_x;
        this.m_left_top_y = in_left_top_y;
        this.m_right_bottom_x = in_right_bottom_x;
        this.m_right_bottom_y = in_right_bottom_y;

        this.update();
    }

    private update(): void {
        this.m_length = this.m_right_bottom_x - this.m_left_top_x;
        this.m_breadth = this.m_left_top_y - this.m_right_bottom_y;

        this.m_area = this.m_length * this.m_breadth;
    }

    left_top_x(): f32 { return this.m_left_top_x; }
    left_top_y(): f32 { return this.m_left_top_y; }

    right_bottom_x(): f32 { return this.m_right_bottom_x; }
    right_bottom_y(): f32 { return this.m_right_bottom_y; }

    length(): f32 {
        return this.m_length;
    }

    breadth(): f32 {
        return this.m_breadth;
    }

    area(): f32 {
        return this.m_area;
    }

    to_polygon(): Polygon {
        const coordinates = new StaticArray<f32>(8);
        const left_top_x = this.m_left_top_x;
        const left_top_y = this.m_left_top_y;
        const right_bottom_x = this.m_right_bottom_x;
        const right_bottom_y = this.m_right_bottom_y;

        coordinates[0] = left_top_x; coordinates[1] = left_top_y;
        coordinates[2] = right_bottom_x; coordinates[3] = left_top_y;
        coordinates[4] = right_bottom_x; coordinates[5] = right_bottom_y;
        coordinates[6] = left_top_x; coordinates[7] = right_bottom_y;

        return new Polygon(coordinates);
    }

    // empty implementations
    // does not need this base class, we use only MBRectangle
    to_bytes(): Uint8Array { return new Uint8Array(0); }
    from_bytes(view: DataView, start: u32): void { }
    bytes_length(): u32 { return 0; }

    toString(): string {
        return `⌜(${this.m_left_top_x}, ${this.m_left_top_y}), (${this.m_right_bottom_x}, ${this.m_right_bottom_y})⌟`;
    }
}

export function do_intersect(a: Rectangle, b: Rectangle): bool {
    const a_left_top_x = a.left_top_x();
    const a_left_top_y = a.left_top_y();
    const a_right_bottom_x = a.right_bottom_x();
    const a_right_bottom_y = a.right_bottom_y();

    const b_left_top_x = b.left_top_x();
    const b_left_top_y = b.left_top_y();
    const b_right_bottom_x = b.right_bottom_x();
    const b_right_bottom_y = b.right_bottom_y();

    if (a_left_top_x > b_right_bottom_x ||
        a_right_bottom_x < b_left_top_x ||
        a_right_bottom_y > b_left_top_y ||
        a_left_top_y < b_right_bottom_y) {
        return false;
    }

    return true;
}

export class MBRectangle extends Rectangle {
    private m_polygon: Polygon | null = null;

    constructor(in_left_top_x: f32 = 0.0, in_left_top_y: f32 = 0.0, in_right_bottom_x: f32 = 0.0, in_right_bottom_y: f32 = 0.0) {
        super(in_left_top_x, in_left_top_y, in_right_bottom_x, in_right_bottom_y);

    }

    polygon(): Polygon | null {
        return this.m_polygon;
    }

    set_polygon(in_polygon: Polygon): void {
        this.m_polygon = in_polygon;
    }

    get_enlargement_area(rectangle_to_fit: MBRectangle): f32 {
        return Mathf.abs(this.get_merged_rectangle(rectangle_to_fit).area() - this.area());
    }

    merge(rectangle_to_merge: MBRectangle): void {
        const merged = this.get_merged_rectangle(rectangle_to_merge);

        this.m_left_top_x = merged.left_top_x();
        this.m_left_top_y = merged.left_top_y();
        this.m_right_bottom_x = merged.right_bottom_x();
        this.m_right_bottom_y = merged.right_bottom_y();

        this.update();
    }

    private get_merged_rectangle(rectangle_to_merge: MBRectangle): Rectangle {
        const left_top_x = this.left_top_x();
        const left_top_y = this.left_top_y();
        const right_bottom_x = this.right_bottom_x();
        const right_bottom_y = this.right_bottom_y();

        const other_left_top_x = rectangle_to_merge.left_top_x();
        const other_left_top_y = rectangle_to_merge.left_top_y();
        const other_right_bottom_x = rectangle_to_merge.right_bottom_x();
        const other_right_bottom_y = rectangle_to_merge.right_bottom_y();

        const left_top_corner_x = left_top_x > other_left_top_x ? other_left_top_x : left_top_x;
        const left_top_corner_y = left_top_y < other_left_top_y ? other_left_top_y : left_top_y;

        const right_bottom_corner_x = right_bottom_x < other_right_bottom_x ? other_right_bottom_x : right_bottom_x;
        const right_bottom_corner_y = right_bottom_y > other_right_bottom_y ? other_right_bottom_y : right_bottom_y;

        return new MBRectangle(left_top_corner_x, left_top_corner_y, right_bottom_corner_x, right_bottom_corner_y);
    }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_RTREE_MBRECTANGLE);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // m_left_top_x
        view.setFloat32(shift, this.m_left_top_x);
        shift += 4;

        // m_left_top_y
        view.setFloat32(shift, this.m_left_top_y);
        shift += 4;

        // m_right_bottom_x
        view.setFloat32(shift, this.m_right_bottom_x);
        shift += 4;

        // m_right_bottom_y
        view.setFloat32(shift, this.m_right_bottom_y);
        shift += 4;

        // m_length
        view.setFloat32(shift, this.m_length);
        shift += 4;

        // m_breadth
        view.setFloat32(shift, this.m_breadth);
        shift += 4;

        // m_area
        view.setFloat32(shift, this.m_area);
        shift += 4;

        const polygon = this.m_polygon;
        if (polygon) {
            let is_polygon = bool_to_bytes(true);
            to_return.set(is_polygon, shift);
            shift += bool_bytes_length();

            // actual polygon data
            to_return.set(polygon.to_bytes(), shift);
            shift += polygon.bytes_length();
        } else {
            let is_polygon = bool_to_bytes(false);
            to_return.set(is_polygon, shift);
            shift += bool_bytes_length();
        }

        return to_return;
    }

    from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_RTREE_MBRECTANGLE) {
            shift += 8;
        } else { return; }

        // m_left_top_x
        this.m_left_top_x = view.getFloat32(shift);
        shift += 4;

        // m_left_top_y
        this.m_left_top_y = view.getFloat32(shift);
        shift += 4;

        // m_right_bottom_x
        this.m_right_bottom_x = view.getFloat32(shift);
        shift += 4;

        // m_right_bottom_y
        this.m_right_bottom_y = view.getFloat32(shift);
        shift += 4;

        // m_length
        this.m_length = view.getFloat32(shift);
        shift += 4;

        // m_breadth
        this.m_breadth = view.getFloat32(shift);
        shift += 4;

        // m_area
        this.m_area = view.getFloat32(shift);
        shift += 4;

        const is_polygon_id = view.getInt32(shift);
        var is_polygon = false;
        if(is_polygon_id == SD_TYPE.SD_TYPE_BOOL) {
            const is_polygon_bytes_length = view.getInt32(shift + 4);
            is_polygon = view.getUint8(shift + 8) == 1;
            shift += is_polygon_bytes_length;
        } else { return; }

        if (is_polygon) {
            // read polygon data
            // get type
            const polygon_type = view.getInt32(shift);
            if (polygon_type == SD_TYPE.SD_TYPE_RTREE_POINT) {
                const new_polygon = new Point();
                new_polygon.from_bytes(view, shift);
                this.m_polygon = new_polygon;
            } else if(polygon_type == SD_TYPE.SD_TYPE_RTREE_EDGE) {
                const new_polygon = new Edge();
                new_polygon.from_bytes(view, shift);
                this.m_polygon = new_polygon;
            } else {
                const new_polygon = new Polygon();
                new_polygon.from_bytes(view, shift);
                this.m_polygon = new_polygon;
            }
        } else {
            this.m_polygon = null;
        }
    }

    bytes_length(): u32 {
        const length = 4 +  // id
                       4 +  // byte length
                       4 +  // left top x
                       4 +  // left top y
                       4 +  // right bottom x
                       4 +  // right bottom y
                       4 +  // length
                       4 +  // breath
                       4 +  // area
                       bool_bytes_length();  // non-null polygon
        const polygon = this.m_polygon;
        if (polygon) {
            return length + polygon.bytes_length();
        } else {
            return length;
        }
    }

    toString(): string {
        const p = this.m_polygon;
        return p ? 
        `⌜(${this.m_left_top_x}, ${this.m_left_top_y}), (${this.m_right_bottom_x}, ${this.m_right_bottom_y})⌟|${p}` : 
        `⌜(${this.m_left_top_x}, ${this.m_left_top_y}), (${this.m_right_bottom_x}, ${this.m_right_bottom_y})⌟|null`;
    }
}
