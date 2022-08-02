import { Graph } from "./navmesh/navmesh_graph";
import { Navmesh, set_bvh_delta, get_bvh_delta } from "./navmesh/navmesh";

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

export function navmesh_get_groups_count(navmesh: Navmesh): i32 {
    return navmesh.get_groups_count();
}

export function navmesh_search_path(navmesh: Navmesh, s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): StaticArray<f32> {
    return navmesh.search_path(s_x, s_y, s_z, e_x, e_y, e_z);
}

export function navmesh_sample(navmesh: Navmesh, x: f32, y: f32, z: f32): StaticArray<f32> {
    return navmesh.sample(x, y, z);
}

export function navmesh_set_bvh_delta(delta: f32): void {
    set_bvh_delta(delta);
}

export function navmesh_get_bvh_delta(): f32 {
    return get_bvh_delta();
}
