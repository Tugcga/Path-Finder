import { Navmesh } from "./navmesh/navmesh";
import { PathFinder} from "./pathfinder";
import { RVOSimulator } from "./rvo/rvo_simulator";
import { Graph } from "./navmesh/navmesh_graph";
import { NavmeshBaker } from "./baker/navmesh_baker";

export const float_array = idof<Float32Array>();
export const int_array = idof<Int32Array>();

/*
Create pathfinder object with default RVO parameters:
    neighbor_dist = 1.0
    max_neighbors = 5
    time_horizon = 0.5
    time_horizon_obst = 0.5
    agent_radius = 0.4
    update_path_find = 1.0
    continuous_moving = false
    move_agents = true
    snap_agents = true
    use_normals = true

Vertices, polygons and sizes arrays can be null. In this case navigation mesh will be infinite horizontal plane
*/
export function create_pathfinder(vertices: Float32Array | null, 
                                  polygons: Int32Array | null, 
                                  sizes: Int32Array | null): PathFinder{
    return create_pathfinder_ext(vertices, polygons, sizes, 1.0, 5, 0.5, 0.5, 0.4, 1.0, false, true, true, true);
}

export function create_pathfinder_ext(vertices: Float32Array | null, 
                                      polygons: Int32Array | null, 
                                      sizes: Int32Array | null,
                                      neighbor_dist: f32,
                                      max_neighbors: i32,
                                      time_horizon: f32,
                                      time_horizon_obst: f32,
                                      agent_radius: f32,
                                      update_path_find: f32,
                                      continuous_moving: bool,
                                      move_agents: bool,
                                      snap_agents: bool,
                                      use_normals: bool): PathFinder{
    if(vertices && polygons && sizes){
        let st_vertices = new StaticArray<f32>(vertices.length);
        let st_polygons = new StaticArray<i32>(polygons.length);
        let st_sizes = new StaticArray<i32>(sizes.length);

        for (let i = 0, len = vertices.length; i < len; i++) {
            unchecked(st_vertices[i] = vertices[i]);
        }

        for (let i = 0, len = polygons.length; i < len; i++) {
            unchecked(st_polygons[i] = polygons[i]);
        }

        for (let i = 0, len = sizes.length; i < len; i++) {
            unchecked(st_sizes[i] = sizes[i]);
        }

        return new PathFinder(st_vertices, st_polygons, st_sizes, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, agent_radius, update_path_find, continuous_moving, move_agents, snap_agents, use_normals);
    }
    else{
        return new PathFinder(null, null, null, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, agent_radius, update_path_find, continuous_moving, move_agents, snap_agents, use_normals);
    }
}

export function create_navmesh(vertices: Float32Array, polygons: Int32Array, sizes: Int32Array): Navmesh{
    let st_vertices = new StaticArray<f32>(vertices.length);
    let st_polygons = new StaticArray<i32>(polygons.length);
    let st_sizes = new StaticArray<i32>(sizes.length);

    for (let i = 0, len = vertices.length; i < len; i++) {
        unchecked(st_vertices[i] = vertices[i]);
    }

    for (let i = 0, len = polygons.length; i < len; i++) {
        unchecked(st_polygons[i] = polygons[i]);
    }

    for (let i = 0, len = sizes.length; i < len; i++) {
        unchecked(st_sizes[i] = sizes[i]);
    }

    return new Navmesh(st_vertices, st_polygons, st_sizes);
}

export function create_rvo_simulator(neighbor_dist: f32,
                                     max_neighbors: i32,
                                     time_horizon: f32,
                                     time_horizon_obst: f32,
                                     agent_radius: f32,
                                     max_speed: f32): RVOSimulator{
    return new RVOSimulator(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, agent_radius, max_speed);
}

/*
Create graph.
    vertex_positions - plane array of floats and contains vertex 3d-coordinates of the first vertex, then of the second and so on [v1_x, v1_y, v1_z, v2_x, v2_y, v2_z, ...]
    vertices - integer names of the vertices in the same order as input positions [v1, v2, v3, ...]
    edges - pairs of graph vertices, for the first edge, then for the second and so on [e1_1, e1_2, e2_1, e2_2, e3_1, e3_2, ...]
*/
export function create_graph(vertex_positions: Float32Array, vertices: Int32Array, edges: Int32Array): Graph {
    //save positions
    let m_positions = new StaticArray<f32>(vertex_positions.length);
    for (let i = 0, len = vertex_positions.length; i < len; i++) {
        unchecked(m_positions[i] = vertex_positions[i]);
    }

    //save names
    let m_vertex_names = new StaticArray<i32>(vertices.length);
    for (let i = 0, len = vertices.length; i < len; i++) {
        unchecked(m_vertex_names[i] = vertices[i]);
    }

    //save edges
    let m_edges = new StaticArray<i32>(edges.length);
    for (let i = 0, len = edges.length; i < len; i++) {
        unchecked(m_edges[i] = edges[i]);
    }
    return new Graph(m_positions, m_vertex_names, m_edges);
}

/*
Create navigation mesh baker object. No input parameters are needed.
*/
export function create_baker(): NavmeshBaker{
    return new NavmeshBaker();
}