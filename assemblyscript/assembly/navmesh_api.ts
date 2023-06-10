import { Graph } from "./navmesh/navmesh_graph";
import { Navmesh, set_bvh_delta, get_bvh_delta } from "./navmesh/navmesh";
import { RTree } from "./navmesh/rtree/rtree";
import { Polygon, Point, Edge } from "./navmesh/rtree/polygon";
import { Rectangle } from "./navmesh/rtree/rectangle";

/*
Create graph.
    vertex_positions - plane array of floats and contains vertex 3d-coordinates of the first vertex, then of the second and so on [v1_x, v1_y, v1_z, v2_x, v2_y, v2_z, ...]
    vertices - integer names of the vertices in the same order as input positions [v1, v2, v3, ...]
    edges - pairs of graph vertices, for the first edge, then for the second and so on [e1_1, e1_2, e2_1, e2_2, e3_1, e3_2, ...]
*/
export function create_graph(vertex_positions: StaticArray<f32>, vertices: StaticArray<i32>, edges: StaticArray<i32>): Graph {
    return new Graph(vertex_positions, vertices, edges);
}

export function create_navmesh(vertices: StaticArray<f32>, polygons: StaticArray<i32>, sizes: StaticArray<i32>): Navmesh {
    return new Navmesh(vertices, polygons, sizes);
}

export function graph_search(graph: Graph, start_vertex: i32, end_vertex: i32): StaticArray<i32> {
    return graph.search(start_vertex, end_vertex);
}

export function graph_to_bytes(graph: Graph): Uint8Array {
    return graph.to_bytes();
}

export function graph_from_bytes(bytes: Uint8Array): Graph {
    let graph = new Graph();
    let view = new DataView(bytes.buffer);
    graph.from_bytes(view, 0);
    return graph;
}

export function navmesh_get_groups_count(navmesh: Navmesh): i32 {
    return navmesh.get_groups_count();
}

export function navmesh_search_path(navmesh: Navmesh, s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): StaticArray<f32> {
    return navmesh.search_path(s_x, s_y, s_z, e_x, e_y, e_z);
}

export function navmesh_search_path_batch(navmesh: Navmesh, coordinates: StaticArray<f32>): StaticArray<f32> {
    return navmesh.search_path_batch(coordinates);
}

export function navmesh_sample(navmesh: Navmesh, x: f32, y: f32, z: f32): StaticArray<f32> {
    return navmesh.sample(x, y, z);
}

export function navmesh_intersect_boundary(navmesh: Navmesh, start_x: f32, start_y: f32, finish_x: f32, finish_y: f32): StaticArray<f32> {
    const t = navmesh.intersect_boundary(start_x, start_y, finish_x, finish_y);
    return StaticArray.fromArray<f32>([start_x + t * (finish_x - start_x), start_y + t * (finish_y - start_y)]);
}

export function navmesh_set_bvh_delta(delta: f32): void {
    set_bvh_delta(delta);
}

export function navmesh_get_bvh_delta(): f32 {
    return get_bvh_delta();
}

export function navmesh_to_bytes(navmesh: Navmesh): Uint8Array {
    return navmesh.to_bytes();
}

export function navmesh_from_bytes(bytes: Uint8Array): Navmesh {
    let nm = new Navmesh();
    nm.from_bytes_array(bytes);

    return nm;
}

// used for export rtree search
// the name of the class does not matter
// because the object without constructor copied in js-side
export class PolygonsSequence {
    count: u32;
    coordinates: StaticArray<StaticArray<f32>>;
}

export function create_rtree(max_nodes: u32): RTree {
    return new RTree(max_nodes >= 3 ? max_nodes : 3);
}

export function rtree_insert_polygon(tree: RTree, coordinates: Array<f32>): void {
    tree.insert(new Polygon(StaticArray.fromArray<f32>(coordinates)));
}

export function rtree_insert_point(tree: RTree, x: f32, y: f32): void {
    tree.insert(new Point(x, y));
}

export function rtree_insert_edge(tree: RTree, s_x: f32, s_y: f32, e_x: f32, e_y: f32): void {
    tree.insert(new Edge(s_x, s_y, e_x, e_y));
}

export function rtree_search(tree: RTree, corner_x: f32, corner_y: f32, other_x: f32, other_y: f32): PolygonsSequence {
    const rect = new Rectangle(
        Mathf.min(corner_x, other_x),
        Mathf.max(corner_y, other_y),
        Mathf.max(corner_x,other_x),
        Mathf.min(corner_y, other_y));

    const result = tree.range_search(rect);
    const to_return = new StaticArray<StaticArray<f32>>(result.length);
    for (let i = 0, len = result.length; i < len; i++) {
        const polygon = result[i];
        to_return[i] = polygon.coordinates();
    }

    return { 
        count: result.length,
        coordinates: to_return 
    };
}

export function rtree_insert_edges(tree: RTree, coordinates: Float32Array): void {
    const count = coordinates.length / 4;
    for (let i = 0; i < count; i++) {
        tree.insert(new Edge(coordinates[4*i], coordinates[4*i + 1],
            coordinates[4*i + 2], coordinates[4*i + 3]));
    }
}

export function rtree_find_intersection(tree: RTree, start_x: f32, start_y: f32, finish_x: f32, fisnih_y: f32): StaticArray<f32> {
    const edge = new Edge(start_x, start_y, finish_x, fisnih_y);
    return tree.find_intersection(edge);
}

export function rtree_to_bytes(tree: RTree): Uint8Array {
    return tree.to_bytes();
}

export function rtree_from_bytes(bytes: Uint8Array): RTree {
    const tree = new RTree();
    tree.from_bytes_array(bytes);

    return tree;
}