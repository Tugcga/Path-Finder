import { Polygon, Edge } from "./polygon";
import { RTreeNode } from "./rtree_node";
import { List } from "../../common/list";
import { Pair } from "../../common/utilities";
import { Rectangle, MBRectangle, do_intersect } from "./rectangle"
import { Serializable, SD_TYPE,
         bool_bytes_length,
         bool_to_bytes } from "../../common/binary_io";

export class RTree extends Serializable {
    private m_max_keys_per_node: u32 = 0;
    private m_min_keys_per_node: u32 = 0;

    // this member required for delete method
    // for now we does not need it
    // when we will need delete nodes, than uncomment this member, delete method
    // and also change the serialization/deserialization process to store this map
    // may be it will require some hashing of the polygon (to use non-reference type as map keys)
    // private m_leaf_mapping: Map<Polygon, RTreeNode>;
    private m_root: RTreeNode | null = null;
    private m_count: u32 = 0;

    constructor(in_max_keys: u32 = 6) {
        super();
        assert(in_max_keys >= 3 , "Max keys per node should be at least 3");

        this.m_max_keys_per_node = in_max_keys;
        this.m_min_keys_per_node = in_max_keys / 2;

        // does not need to delete nodes
        // this.m_leaf_mapping = new Map<Polygon, RTreeNode>();
    }

    insert(polygon: Polygon): void {
        const max_keys = this.m_max_keys_per_node;
        const new_node = new RTreeNode(max_keys, null);
        new_node.set_mbr(polygon.get_containing_rectangle());

        // ignore this
        // this.m_leaf_mapping.set(polygon, new_node);
        this.insert_to_leaf(new_node);
        this.m_count += 1;
    }

    private insert_to_leaf(new_node: RTreeNode): void {
        const root = this.m_root;
        if (root) {
            const leaf_to_insert = this.find_insertion_leaf(root, new_node);
            this.inster_and_split(leaf_to_insert, new_node);
        } else {
            const max_keys = this.m_max_keys_per_node;
            this.m_root = new RTreeNode(max_keys, null);
            const root_again = this.m_root;
            if (root_again) {
                root_again.add_child(new_node);
            }
        }
    }

    private inster_internal_node(current_node: RTreeNode, internal_node: RTreeNode): void {
        if (current_node.height() == internal_node.height() + 1) {
            this.inster_and_split(current_node, internal_node);
        } else {
            const next_node = current_node.get_minimum_enlargement_area_mbr(internal_node.mbr());
            if (next_node) {
                this.inster_internal_node(next_node, internal_node);
            }
        }
    }

    private find_insertion_leaf(node: RTreeNode, new_node: RTreeNode): RTreeNode {
        if (node.is_leaf()) {
            return node;
        } else {
            const new_node_mbr = new_node.mbr();
            if (new_node_mbr) {
                const next_node = node.get_minimum_enlargement_area_mbr(new_node_mbr);
                if (next_node) {
                    return this.find_insertion_leaf(next_node, new_node);
                } else {
                    // should not happens
                    return node;
                }
            } else {
                // also should not happens
                return node;
            }
        }
    }

    private inster_and_split(node: RTreeNode, new_value: RTreeNode): void {
        const max_keys = this.m_max_keys_per_node;
        const min_keys = this.m_min_keys_per_node;
        if (node.key_count() < max_keys) {
            node.add_child(new_value);
            this.expand_ancestor_mbrs(node);
        } else {
            // e is a list of all shildren nodes and the new one
            const e = new List<RTreeNode>();
            e.push(new_value);
            const node_children_count = node.key_count();
            for (let i: u32 = 0; i < node_children_count; i++) {
                const child = node.child(i);
                if (child) {
                    e.push(child);
                }
            }

            const distant_pairs = this.get_distance_pair(e);
            const e1 = new RTreeNode(max_keys, null);
            const e2 = new RTreeNode(max_keys, null);

            e1.add_child(distant_pairs.x());
            e2.add_child(distant_pairs.y());

            // remove from e elements from pair
            e.pop_value(distant_pairs.x());
            e.pop_value(distant_pairs.y());

            while (e.length > 0) {
                const current = e.pop_last();
                const current_mbr = current.mbr();
                const e1_mbr = e1.mbr();
                const e2_mbr = e2.mbr();
                if (current_mbr && e1_mbr && e2_mbr) {
                    const left_enlargenent_area = e1_mbr.get_enlargement_area(current_mbr);
                    const right_enlargement_area = e2_mbr.get_enlargement_area(current_mbr);
                    if (left_enlargenent_area == right_enlargement_area) {
                        const left_area = e1_mbr.area();
                        const right_area = e2_mbr.area();

                        if (left_area == right_area) {
                            if (e1.key_count() < e2.key_count()) {
                                e1.add_child(current);
                            } else {
                                e2.add_child(current);
                            }
                        } else if (left_area < right_area) {
                            e1.add_child(current);
                        } else {
                            e2.add_child(current);
                        }
                    } else if (left_enlargenent_area < right_enlargement_area) {
                        e1.add_child(current);
                    } else {
                        e2.add_child(current);
                    }
                }

                const remainig = e.length;
                if (e1.key_count() + remainig == min_keys) {
                    for (let i = 0, len = e.length; i < len; i++) {
                        e1.add_child(e[i]);
                    }
                    e.reset();
                } else if (e2.key_count() + remainig == min_keys) {
                    for (let i = 0, len = e.length; i < len; i++) {
                        e2.add_child(e[i]);
                    }
                    e.reset();
                }
            }

            const parent = node.parent();
            if (parent) {
                parent.set_child(node.index(), e1);
                this.inster_and_split(parent, e2);
            } else {
                const root = new RTreeNode(max_keys, null);
                root.add_child(e1);
                root.add_child(e2);
                this.m_root = root;
            }
        }
    }

    private expand_ancestor_mbrs(node: RTreeNode): void {
        var local_parent = node.parent();
        var local_node = node;
        while (local_parent) {
            const local_parent_mbr = local_parent.mbr();
            if (local_parent_mbr) {
                const node_mbr = node.mbr();
                if (node_mbr) {
                    local_parent_mbr.merge(node_mbr);
                    local_parent.set_height(local_node.height() + 1);

                    local_node = local_parent;
                    local_parent = local_parent.parent();
                }
            }
        }
    }

    private get_distance_pair(all_entries: List<RTreeNode>): Pair<RTreeNode> {
        var max_area: f32 = f32.MIN_VALUE;
        var max_i: u32 = 0;
        var max_j: u32 = 0;
        const count = all_entries.length;
        for (let  i = 0; i < count; i++) {
            const entry_i = all_entries[i];
            for (let j = i + 1; j < count; j++) {
                const entry_j = all_entries[j];

                const entry_i_mbr = entry_i.mbr();
                const entry_j_mbr = entry_j.mbr();
                if (entry_i_mbr && entry_j_mbr) {
                    const current_area = entry_i_mbr.get_enlargement_area(entry_j_mbr);
                    if (current_area > max_area) {
                        max_area = current_area;
                        max_i = i;
                        max_j = j;
                    }
                }
            }
        }

        return new Pair<RTreeNode>(all_entries[max_i], all_entries[max_j]);
    }

    // does not required
    /*exists(search_polygon: Polygon): bool {
        return this.m_leaf_mapping.has(search_polygon);
    }*/

    range_search(search_rectangle: Rectangle): List<Polygon> {
        const result = new List<Polygon>();
        const root = this.m_root;

        if (root) {
            return this.range_search_ext(root, search_rectangle, result);
        } else {
            return new List<Polygon>();
        }
    }

    private range_search_ext(current: RTreeNode, search_rectangle: Rectangle, result: List<Polygon>): List<Polygon> {
        if (current.is_leaf()) {
            for (let i: u32 = 0, len = current.key_count(); i < len; i++) {
                const node = current.child(i);
                if (node) {
                    const node_mbr = node.mbr();
                    if (node_mbr) {
                        if (do_intersect(node_mbr, search_rectangle)) {
                            const p = node_mbr.polygon();
                            if (p) {
                                result.push(p);
                            }
                        }
                    }
                }
            }
        }

        for (let i: u32 = 0, len = current.key_count(); i < len; i++) {
            const node = current.child(i);
            if (node) {
                const node_mbr = node.mbr();
                if (node_mbr) {
                    if (do_intersect(node_mbr, search_rectangle)) {
                        this.range_search_ext(node, search_rectangle, result);
                    }
                }
            }
        }

        return result;
    }

    // does not required
    /*delete(polyglo: Polygon): void {
        const root = this.m_root;
        if (root) {
            assert(this.exists(polygon), "Given polygon do not belong to this tree");

            const node_to_delete = this.m_leaf_mapping.get(polygon);

            this.delete_node(node_to_delete);
            this.condense_tree(node_to_delete.parent());

            if (root.key_count() == 1 && !root.is_leaf()) {
                root = root.child(0);
                root.set_parent = null;

                this.m_root = root;
            }

            this.m_leaf_mapping.remove(polygon);
            this.m_count -= 1;

            if (this.m_count == 0) {
                this.m_root = null;
            }
        }
    }*/

    private delete_node(node_to_delete: RTreeNode): void {
        const node_parent = node_to_delete.parent();
        const node_index = node_to_delete.index();
        if (node_parent) {
            // remove child with node_index from parent children
            for (let i = node_index, len = parent.key_count() - 1; i < len; i++) {
                node_parent.set_child(i, node_parent.child(i + 1));
            }

            node_parent.decrease_key_count();

            // decrease indices of all children from index and above
            for (let i = node_index, len = node_parent.key_count(); i < len; i++) {
                const child = node_parent.child(i);
                if (child) {
                    child.set_index(child.index() - 1);
                }
            }
        }
    }

    private condense_tree(updated_leaf: RTreeNode | null): void {
        var local_current = updated_leaf;
        const to_reinsert = new List<RTreeNode>();
        const root = this.m_root;
        const miin_keys = this.m_min_keys_per_node;

        while (local_current != root) {
            const parent = local_current.parent();
            if (parent) {
                if (local_current.key_count() < min_keys) {
                    this.delete_node(local_current);
                    for (let i: u32 = 0, len = local_current.key_count(); i < len; i++) {
                        const node = local_current.child(i);
                        if (node) {
                            to_reinsert.push(node);
                        }
                    }
                } else {
                    this.shrink_mbr(local_current);
                }

                local_current = parent;
            } else {
                local_current = root;
            }
        }

        if (local_current.key_count() > 0) {
            this.shrink_mbr(local_current);
        }

        while (to_reinsert.length > 0) {
            const node = to_reinsert.pop_last();

            if (node.height() > 0) {
                this.inster_internal_node(node);
            } else {
                this.insert_to_leaf(node);
            }
        }
    }

    private shrink_mbr(current: RTreeNode): void {
        const child = current.child(0);
        if (child) {
            current.set_mbr(new MBRectangle(child.mbr()));

            const count = current.key_count();
            const current_mbr = current.mbr();
            for (let i = 1; i < count; i++) {
                const node = current.child(i);
                current_mbr.merge(node.mbr());
            }
        }
    }

    clear(): void {
        this.m_root = null;
        // does not required
        // this.m_leaf_mapping.clear();
        this.m_count = 0;
    }

    // return t-parameter of the point in the input edge, where it intersects with polygons in the tree
    find_intersection_t(start_x: f32, start_y: f32, finish_x: f32, finish_y: f32): f32 {
        const edge_rect = new Rectangle(Mathf.min(start_x, finish_x), Mathf.max(start_y, finish_y),
                                        Mathf.max(start_x, finish_x), Mathf.min(start_y, finish_y));

        const polygons = this.range_search(edge_rect);
        const poly_count = polygons.length;
        var closed_t: f32 = 1.0;
        for (let i = 0; i < poly_count; i++) {
            const polygon = polygons[i];
            const corners = polygon.corners();
            if (corners > 1) {  // ignore points
                const lines_count = corners == 2 ? 1 : corners;
                const polygon_coordinates = polygon.coordinates();
                const coordinates_count = polygon_coordinates.length;
                for (let line_index = 0; line_index < lines_count; line_index++) {
                    const line_start_x = polygon_coordinates[2*line_index];
                    const line_start_y = polygon_coordinates[2*line_index + 1];
                    const line_end_x = polygon_coordinates[(2*line_index + 2) % coordinates_count];
                    const line_end_y = polygon_coordinates[(2*line_index + 3) % coordinates_count];

                    const in_edge_x = line_end_x - line_start_x;
                    const in_edge_y = line_end_y - line_start_y;

                    const denum = (finish_x - start_x) * in_edge_y - (finish_y - start_y) * in_edge_x;

                    const to_start_x = line_start_x - start_x;
                    const to_start_y = line_start_y - start_y;
                    const num = in_edge_y * to_start_x - in_edge_x * to_start_y;

                    if (Mathf.abs(denum) >= Mathf.abs(num)) {
                        const t = num / denum;

                        // calc the point in the edge
                        const p_x = start_x + t * (finish_x - start_x);
                        const p_y = start_y + t * (finish_y - start_y);

                        // calculate two vectors from in_edge endpoints to this point
                        const line_start_to_point_x = p_x - line_start_x;
                        const line_start_to_point_y = p_y - line_start_y;

                        const line_end_to_point_x = p_x - line_end_x;
                        const line_end_to_point_y = p_y - line_end_y;

                        // calculate dot product of these two vectors
                        const d = line_start_to_point_x * line_end_to_point_x + line_start_to_point_y * line_end_to_point_y;
                        if (d < 0.0) {
                            // point inside the interval
                            if (t >= 0.0 && t < closed_t) {
                                closed_t = t;
                            }
                        }
                    }
                }
            }
        }

        return closed_t;
    }

    // if there are no intersections, return the end point of the edge
    find_intersection(edge: Edge): StaticArray<f32> {
        // find polygons within edge rectangle
        const polygons = this.range_search(edge.get_containing_rectangle(false));
        const poly_count = polygons.length;
        var closed_t: f32 = 1.0;
        for (let i = 0; i < poly_count; i++) {
            const polygon = polygons[i];
            const corners = polygon.corners();
            if (corners > 1) {  // ignore points
                const lines_count = corners == 2 ? 1 : corners;
                const polygon_coordinates = polygon.coordinates();
                const coordinates_count = polygon_coordinates.length;
                for (let line_index = 0; line_index < lines_count; line_index++) {
                    const line_start_x = polygon_coordinates[2*line_index];
                    const line_start_y = polygon_coordinates[2*line_index + 1];
                    const line_end_x = polygon_coordinates[(2*line_index + 2) % coordinates_count];
                    const line_end_y = polygon_coordinates[(2*line_index + 3) % coordinates_count];

                    const in_edge_x = line_end_x - line_start_x;
                    const in_edge_y = line_end_y - line_start_y;

                    const denum = edge.to_x() * in_edge_y - edge.to_y() * in_edge_x;

                    const to_start_x = line_start_x - edge.start_x();
                    const to_start_y = line_start_y - edge.start_y();
                    const num = in_edge_y * to_start_x - in_edge_x * to_start_y;

                    if (Mathf.abs(denum) >= Mathf.abs(num)) {
                        const t = num / denum;

                        // calc the point in the edge
                        const p_x = start_x + t * (finish_x - start_x);
                        const p_y = start_y + t * (finish_y - start_y);

                        // calculate two vectors from in_edge endpoints to this point
                        const line_start_to_point_x = p_x - line_start_x;
                        const line_start_to_point_y = p_y - line_start_y;

                        const line_end_to_point_x = p_x - line_end_x;
                        const line_end_to_point_y = p_y - line_end_y;

                        // calculate dot product of these two vectors
                        const d = line_start_to_point_x * line_end_to_point_x + line_start_to_point_y * line_end_to_point_y;
                        if (d < 0.0) {
                            // point inside the interval
                            if (t >= 0.0 && t < closed_t) {
                                closed_t = t;
                            }
                        }
                    }
                }
            }
        }

        return StaticArray.fromArray<f32>([edge.x(closed_t), edge.y(closed_t)]);
    }

    to_bytes(): Uint8Array {
        const bytes_length = this.bytes_length();
        let to_return = new Uint8Array(bytes_length);
        let view = new DataView(to_return.buffer);
        // id
        view.setInt32(0, SD_TYPE.SD_TYPE_RTREE);
        let shift = 4;

        // bytes length
        view.setInt32(shift, bytes_length);
        shift += 4;

        // m_max_keys_per_node
        view.setUint32(shift, this.m_max_keys_per_node);
        shift += 4;

        // m_min_keys_per_node
        view.setUint32(shift, this.m_min_keys_per_node);
        shift += 4;

        const root = this.m_root;
        if (root) {
            const is_root_bytes = bool_to_bytes(true);
            to_return.set(is_root_bytes, shift);
            shift += bool_bytes_length();

            const root_bytes = root.to_bytes();
            to_return.set(root_bytes, shift);
            shift += root.bytes_length();
        } else {
            const is_root_bytes = bool_to_bytes(false);
            to_return.set(is_root_bytes, shift);
            shift += bool_bytes_length();
        }

        // m_count
        view.setUint32(shift, this.m_count);
        shift += 4;

        return to_return;
    }

    from_bytes_array(bytes: Uint8Array): void {
        const view = new DataView(bytes.buffer);
        this.from_bytes(view, 0);
    }

    from_bytes(view: DataView, start: u32): void {
        const id = view.getInt32(start + 0);
        const bytes_length = view.getInt32(start + 4);
        let shift = start + 0;
        if(id == SD_TYPE.SD_TYPE_RTREE) {
            shift += 8;
        } else { return; }

        this.m_max_keys_per_node = view.getUint32(shift);
        shift += 4;

        this.m_min_keys_per_node = view.getUint32(shift);
        shift += 4;

        const is_root_id = view.getInt32(shift);
        var is_root = false;
        if(is_root_id == SD_TYPE.SD_TYPE_BOOL) {
            const is_root_bytes_length = view.getInt32(shift + 4);
            is_root = view.getUint8(shift + 8) == 1;
            shift += is_root_bytes_length;
        } else { return; }

        if (is_root) {
            const new_root = new RTreeNode();
            new_root.from_bytes(view, shift);
            shift += view.getInt32(shift + 4);
            this.m_root = new_root;
        }

        this.m_count = view.getUint32(shift);
        shift += 4;
    }

    bytes_length(): u32 {
        var to_return = 4 + 4;  // id and byte length
        to_return += 4;  // m_max_keys_per_node
        to_return += 4;  // m_min_keys_per_node
        to_return += bool_bytes_length();  // m_root is null or not

        const root = this.m_root;
        if (root) {
            to_return += root.bytes_length();
        }

        to_return += 4;  // m_count

        return to_return;
    }

    toString(): string {
        const root = this.m_root;
        if (root) {
            return `[${this.m_count}]` + root.toString();
        } else {
            return "Empty";
        }
    }
}