import { MBRectangle } from "./rectangle";
import { List } from "../../common/list";
import { Serializable, SD_TYPE,
         bool_bytes_length,
         bool_to_bytes } from "../../common/binary_io";

export class RTreeNode extends Serializable {
    private m_index: u32 = 0;
    private m_height: u32 = 0;
    private m_mb_rectangle: MBRectangle | null = null;
    private m_key_count: u32 = 0;

    private m_is_parent: bool = false;
    private m_parent: RTreeNode | null = null;
    private m_children: StaticArray<RTreeNode | null>;

    constructor(max_keys_per_node: u32 = 0, parent: RTreeNode | null = null) {
        super();
        this.m_parent = parent;
        if (parent) {
            this.m_is_parent = true;
        }
        this.m_children = new StaticArray<RTreeNode | null>(max_keys_per_node);
    }

    is_leaf(): bool {
        const mbr = this.m_mb_rectangle;
        if (mbr) {
            const mbr_polygon = mbr.polygon();
            if (mbr_polygon) {
                return true;
            }

            const children = this.m_children;
            if (this.m_key_count == 0) {
                return false;
            }

            const child = children[0];
            if (child) {
                const ch_mbr = child.mbr();
                if (ch_mbr) {
                    const ch_mbr_polygon = ch_mbr.polygon();
                    if (ch_mbr_polygon) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    add_child(in_child: RTreeNode): void {
        this.set_child(this.m_key_count, in_child);

        this.m_key_count += 1;
    }

    decrease_key_count(): void {
        this.m_key_count -= 1;
    }

    // called in deserialization process
    set_parent_if_required(in_parent: RTreeNode): void {
        if (this.m_is_parent) {
            this.m_parent = in_parent;
        }
    }

    set_parent(in_parent: RTreeNode): void {
        this.m_parent = in_parent;
        this.m_is_parent = true;
    }

    set_index(in_index: u32): void {
        this.m_index = in_index;
    }

    set_mbr(in_mbr: MBRectangle): void {
        this.m_mb_rectangle = in_mbr;
    }

    set_height(in_height: u32): void {
        this.m_height = in_height;
    }

    mbr(): MBRectangle | null {
        return this.m_mb_rectangle;
    }

    height(): u32 {
        return this.m_height;
    }

    parent(): RTreeNode | null {
        return this.m_parent;
    }

    key_count(): u32 {
        return this.m_key_count;
    }

    child(index: u32): RTreeNode | null {
        return this.m_children[index];
    }

    index(): u32 {
        return this.m_index;
    }

    set_child(index: u32, in_child: RTreeNode): void {
        const children = this.m_children;
        children[index] = in_child;

        in_child.set_parent(this);
        in_child.set_index(index);

        const child_mbr = in_child.mbr();
        if (child_mbr) {
            const mbr = this.m_mb_rectangle;
            if (mbr) {
                mbr.merge(child_mbr);
            } else {
                this.m_mb_rectangle = new MBRectangle(child_mbr.left_top_x(), child_mbr.left_top_y(), child_mbr.right_bottom_x(), child_mbr.right_bottom_y());
            }

            this.m_height = in_child.height() + 1;
        }
    }

    get_minimum_enlargement_area_mbr(new_polygon: MBRectangle): RTreeNode | null {
        if (this.m_key_count == 0) {
            return null;
        }

        const children = this.m_children;
        var min_index = 0;
        var child = children[0];
        if (child) {
            var child_mbr = child.mbr();
            if (child_mbr) {
                var min_area = child_mbr.get_enlargement_area(new_polygon);

                for (let i: u32 = 1, len = this.m_key_count; i < len; i++) {
                    child = children[i];
                    if (child) {
                        child_mbr = child.mbr();
                        if (child_mbr) {
                            const new_area = child_mbr.get_enlargement_area(new_polygon);
                            if (new_area < min_area) {
                                min_area = new_area;
                                min_index = i;
                            }
                        }
                    }
                }
            } else {
                return null;
            }
        } else {
            return null;
        }

        return children[min_index];
    }

    geather_nodes(nodes: List<RTreeNode | null>): void {
        nodes.push(this);

        const children = this.m_children;
        for (let i = 0, len = this.m_key_count; i < len; i++) {
            const child = children[i];
            if (child) {
                child.geather_nodes(nodes);
            }
        }
    }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_RTREE_NODE);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // m_index
        view.setUint32(shift, this.m_index);
        shift += 4;

        // m_height
        view.setUint32(shift, this.m_height);
        shift += 4;

        // is mbr null
        const mbr = this.m_mb_rectangle;
        const is_mbr = mbr ? true : false;
        const is_mbr_bytes = bool_to_bytes(is_mbr);
        to_return.set(is_mbr_bytes, shift);
        shift += bool_bytes_length();

        // mbr
        if (mbr) {
            const mbr_bytes = mbr.to_bytes();
            to_return.set(mbr_bytes, shift);
            shift += mbr.bytes_length();
        }

        // m_key_count
        view.setUint32(shift, this.m_key_count);
        shift += 4;

        // is_parent
        const is_parent_bytes = bool_to_bytes(this.m_is_parent);
        to_return.set(is_parent_bytes, shift);
        shift += bool_bytes_length();

        // next children
        // total length as u32
        const childrent_total: u32 = this.m_children.length;
        view.setUint32(shift, childrent_total);
        shift += 4;

        // next for each valid child
        for (let i: u32 = 0, len = this.m_key_count; i < len; i++) {
            const child = this.m_children[i];
            if (child) {
                const child_bytes = child.to_bytes();
                to_return.set(child_bytes, shift);
                shift += child.bytes_length();
            }
        }

        return to_return;
    }

    from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_RTREE_NODE) {
            shift += 8;
        } else { return; }

        this.m_index = view.getUint32(shift);
        shift += 4;

        this.m_height = view.getUint32(shift);
        shift += 4;

        const is_mbr_id = view.getInt32(shift);
        var is_mbr = false;
        if(is_mbr_id == SD_TYPE.SD_TYPE_BOOL) {
            const is_mbr_bytes_length = view.getInt32(shift + 4);
            is_mbr = view.getUint8(shift + 8) == 1;
            shift += is_mbr_bytes_length;
        } else { return; }
        if (is_mbr) {
            const new_mbr = new MBRectangle();
            new_mbr.from_bytes(view, shift);
            shift += view.getInt32(shift + 4);

            this.m_mb_rectangle = new_mbr;
        }

        this.m_key_count = view.getUint32(shift);
        shift += 4;

        const is_parent_id = view.getInt32(shift);
        this.m_is_parent = false;
        if(is_parent_id == SD_TYPE.SD_TYPE_BOOL) {
            const is_parent_bytes_length = view.getInt32(shift + 4);
            this.m_is_parent = view.getUint8(shift + 8) == 1;
            shift += is_parent_bytes_length;
        } else { return; }

        const total_children = view.getUint32(shift);
        shift += 4;
        this.m_children = new StaticArray<RTreeNode | null>(total_children);
        for (let i: u32 = 0, len = this.m_key_count; i < len; i++) {
            const new_child = new RTreeNode();
            new_child.from_bytes(view, shift);
            shift += view.getInt32(shift + 4);

            new_child.set_parent_if_required(this);
            this.m_children[i] = new_child;
        }
    }

    bytes_length(): u32 {
        var to_return = 4 + 4;  // id and byte length
        to_return += 4;  // m_index
        to_return += 4;  // m_height
        to_return += bool_bytes_length();  // is mbr
        const mbr = this.m_mb_rectangle;
        if (mbr) {
            to_return += mbr.bytes_length();
        }

        to_return += 4;  // key_count
        to_return += bool_bytes_length();  // is parent
        // does not store actual parent data, only the fact tha it exists or not

        to_return += 4;  // the total length of m_children (with possibly null values)
        
        for (let i: u32 = 0, len = this.m_key_count; i < len; i++) {
            const child = this.m_children[i];
            if (child) {
                to_return += child.bytes_length();
            }
        }

        return to_return;
    }

    toString(): string {
        var to_print = "";
        const children = this.m_children;
        const count = this.m_key_count;
        for (let i: u32 = 0; i < count; i++) {
            const child = children[i];
            if (child) {
                to_print += i.toString() + ":" + child.toString();
            } else {
                to_print += i.toString() + ":empty";
            }
        }

        const mbr = this.m_mb_rectangle;
        if (mbr) {
            const mbr_poly = mbr.polygon();
            if (mbr_poly) {
                to_print += "|" + mbr_poly.toString();
            }
        }
        return `{${to_print}}`;
    }
}