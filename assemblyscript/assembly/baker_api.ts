import { NavmeshBaker } from "./baker/navmesh_baker";

/*
Create navigation mesh baker object. No input parameters are needed.
*/
export function create_baker(): NavmeshBaker{
    return new NavmeshBaker();
}

export function baker_add_geometry(baker: NavmeshBaker, vertices: StaticArray<f32>, polygons: StaticArray<i32>, sizes: StaticArray<i32>): void {
    baker.add_geometry(vertices, polygons, sizes);
}

export function baker_bake(baker: NavmeshBaker, cell_size: f64, cell_height: f64, agent_height: f64, agent_radius: f64, agent_max_climb: f64, agent_max_slope: f64, region_min_size: i32, region_merge_size: i32, edge_max_len: f64, edge_max_error: f64, verts_per_poly: i32, detail_sample_distance: f64, detail_sample_maximum_error: f64): bool {
    return baker.bake(cell_size, cell_height, agent_height, agent_radius, agent_max_climb, agent_max_slope, region_min_size, region_merge_size, edge_max_len, edge_max_error, verts_per_poly, detail_sample_distance, detail_sample_maximum_error);
}

export function baker_get_navmesh_vertices(baker: NavmeshBaker): StaticArray<f64> {
    return baker.get_navmesh_vertices();
}

export function baker_get_navmesh_polygons(baker: NavmeshBaker): StaticArray<i32> {
    return baker.get_navmesh_polygons();
}

export function baker_get_navmesh_sizes(baker: NavmeshBaker): StaticArray<i32> {
    return baker.get_navmesh_sizes();
}