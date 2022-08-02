import { PathFinder } from "./pathfinder";
import { Navmesh } from "./navmesh/navmesh";
import { RVOSimulator } from "./rvo/rvo_simulator";

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
export function create_pathfinder(vertices: StaticArray<f32> | null, 
                                  polygons: StaticArray<i32> | null, 
                                  sizes: StaticArray<i32> | null): PathFinder{
    return create_pathfinder_ext(vertices, polygons, sizes, 1.0, 5, 0.5, 0.5, 0.4, 1.0, false, true, true, true);
}

export function create_pathfinder_ext(vertices: StaticArray<f32> | null, 
                                      polygons: StaticArray<i32> | null, 
                                      sizes: StaticArray<i32> | null,
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
        return new PathFinder(vertices, polygons, sizes, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, agent_radius, update_path_find, continuous_moving, move_agents, snap_agents, use_normals);
    }
    else{
        return new PathFinder(null, null, null, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, agent_radius, update_path_find, continuous_moving, move_agents, snap_agents, use_normals);
    }
}

export function pathfinder_add_agent(pathfinder: PathFinder, position_x: f32, position_y: f32, position_z: f32, radius: f32, speed: f32): void {
    pathfinder.add_agent(position_x, position_y, position_z, radius, speed);
}

export function pathfinder_delete_agent(pathfinder: PathFinder, agent_id: i32): void {
    pathfinder.delete_agent(agent_id);
}

export function pathfinder_set_agent_destination(pathfinder: PathFinder, agent_id: i32, position_x: f32, position_y: f32, position_z: f32): bool {
    return pathfinder.set_agent_destination(agent_id, position_x, position_y, position_z);
}

export function pathfinder_update(pathfinder: PathFinder, delta_time: f32): void {
    pathfinder.update(delta_time);
}

export function pathfinder_get_default_agent_radius(pathfinder: PathFinder): f32 {
    return pathfinder.get_default_agent_radius();
}

export function pathfinder_get_all_agents_positions(pathfinder: PathFinder): StaticArray<f32> {
    return pathfinder.get_all_agents_positions();
}

export function pathfinder_get_all_agents_velocities(pathfinder: PathFinder): StaticArray<f32> {
    return pathfinder.get_all_agents_velocities();
}

export function pathfinder_get_agent_path(pathfinder: PathFinder, agent_id: i32): Float32Array {
    return pathfinder.get_agent_path(agent_id);
}

export function pathfinder_get_all_agents_activities(pathfinder: PathFinder): StaticArray<bool> {
    return pathfinder.get_all_agents_activities();
}

export function pathfinder_get_agent_activity(pathfinder: PathFinder, agent_id: i32): bool {
    return pathfinder.get_agent_activity(agent_id);
}

export function pathfinder_get_agent_velocity(pathfinder: PathFinder, agent_id: i32): StaticArray<f32> {
    return pathfinder.get_agent_velocity(agent_id);
}

export function pathfinder_get_agent_position(pathfinder: PathFinder, agent_id: i32): StaticArray<f32> {
    return pathfinder.get_agent_position(agent_id);
}

export function pathfinder_get_agents_count(pathfinder: PathFinder): i32 {
    return pathfinder.get_agents_count();
}

export function pathfinder_get_agents_id(pathfinder: PathFinder): StaticArray<i32> {
    return pathfinder.get_agents_id();
}

export function pathfinder_get_active_agents_count(pathfinder: PathFinder): i32 {
    return pathfinder.get_active_agents_count();
}

export function pathfinder_search_path(pathfinder: PathFinder, s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): Float32Array {
    return pathfinder.search_path(s_x, s_y, s_z, e_x, e_y, e_z);
}

export function pathfinder_sample(pathfinder: PathFinder, x: f32, y: f32, z: f32): StaticArray<f32> {
    return pathfinder.sample(x, y, z);
}

export function pathfinder_get_neighbor_dist(pathfinder: PathFinder): f32 {
    return pathfinder.get_neighbor_dist();
}

export function pathfinder_get_max_neighbors(pathfinder: PathFinder): i32 {
    return pathfinder.get_max_neighbors();
}

export function pathfinder_get_time_horizon(pathfinder: PathFinder): f32 {
    return pathfinder.get_time_horizon();
}

export function pathfinder_get_time_horizon_obst(pathfinder: PathFinder): f32 {
    return pathfinder.get_time_horizon_obst();
}

export function pathfinder_get_update_path_find(pathfinder: PathFinder): f32 {
    return pathfinder.get_update_path_find();
}

export function pathfinder_set_update_path_find(pathfinder: PathFinder, value: f32): void {
    pathfinder.set_update_path_find(value);
}

export function pathfinder_get_continuous_moving(pathfinder: PathFinder): bool {
    return pathfinder.get_continuous_moving();
}

export function pathfinder_set_continuous_moving(pathfinder: PathFinder, value: bool): void {
    pathfinder.set_continuous_moving(value);
}

export function pathfinder_get_move_agents(pathfinder: PathFinder): bool {
    return pathfinder.get_move_agents();
}

export function pathfinder_set_move_agents(pathfinder: PathFinder, value: bool): void {
    pathfinder.set_move_agents(value);
}

export function pathfinder_get_snap_agents(pathfinder: PathFinder): bool {
    return pathfinder.get_snap_agents();
}

export function pathfinder_set_snap_agents(pathfinder: PathFinder, value: bool): void {
    pathfinder.set_snap_agents(value);
}

export function pathfinder_get_use_normals(pathfinder: PathFinder): bool {
    return pathfinder.get_use_normals();
}

export function pathfinder_set_use_normals(pathfinder: PathFinder, value: bool): void {
    pathfinder.set_use_normals(value);
}

export function pathfinder_get_rvo_simulator(pathfinder: PathFinder, group: i32): RVOSimulator | null {
    return pathfinder.get_rvo_simulator(group);
}

export function pathfinder_get_navmesh(pathfinder: PathFinder): Navmesh | null {
    return pathfinder.get_navmesh();
}