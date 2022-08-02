import { RVOSimulator } from "./rvo/rvo_simulator";
import { Vector2 } from "./common/vector2";

export function vector2_x(vector: Vector2): f32 {
    return vector.x();
}

export function vector2_y(vector: Vector2): f32 {
    return vector.y();
}

export function create_rvo_simulator(neighbor_dist: f32,
                                     max_neighbors: i32,
                                     time_horizon: f32,
                                     time_horizon_obst: f32,
                                     agent_radius: f32,
                                     max_speed: f32): RVOSimulator{
    return new RVOSimulator(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, agent_radius, max_speed);
}

export function rvo_add_agent(rvo: RVOSimulator, position_x: f32, position_y: f32): i32 {
    return rvo.add_agent(position_x, position_y);
}

export function rvo_add_agent_ext(rvo: RVOSimulator, position_x: f32, position_y: f32, velocity_x: f32, velocity_y: f32, radius: f32, neighbor_dist: f32, max_neighbors: i32, time_horizon: f32, time_horizon_obst: f32, max_speed: f32): i32 {
    return rvo.add_agent_ext(position_x, position_y, velocity_x, velocity_y, radius, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, max_speed);
}

export function rvo_delete_agent(rvo: RVOSimulator, agent_index: i32): void {
    rvo.delete_agent(agent_index);
}

export function rvo_get_agents_count(rvo: RVOSimulator): i32 {
    return rvo.get_agents_count();
}

export function rvo_add_obstacle_array(rvo: RVOSimulator, vertices: StaticArray<f32>): i32 {
    return rvo.add_obstacle_array(vertices);
}

export function rvo_do_step(rvo: RVOSimulator, delta_time: f32, move_agents: bool = true): void {
    rvo.do_step(delta_time, move_agents);
}

export function rvo_get_agent_max_neighbors(rvo: RVOSimulator, agent_index: i32): i32 {
    return rvo.get_agent_max_neighbors(agent_index);
}

export function rvo_get_agent_max_speed(rvo: RVOSimulator, agent_index: i32): f32 {
    return rvo.get_agent_max_speed(agent_index);
}

export function rvo_get_agent_position(rvo: RVOSimulator, agent_index: i32): Vector2 {
    return rvo.get_agent_position(agent_index);
}

export function rvo_get_agents_positions(rvo: RVOSimulator): StaticArray<f32> {
    return rvo.get_agents_positions();
}

export function rvo_get_agent_pref_velocity(rvo: RVOSimulator, agent_index: i32): Vector2 {
    return rvo.get_agent_pref_velocity(agent_index);
}

export function rvo_get_agent_radius(rvo: RVOSimulator, agent_index: i32): f32 {
    return rvo.get_agent_radius(agent_index);
}

export function rvo_get_agent_time_horizon(rvo: RVOSimulator, agent_index: i32): f32 {
    return rvo.get_agent_time_horizon(agent_index);
}

export function rvo_get_agent_time_horizon_obst(rvo: RVOSimulator, agent_index: i32): f32 {
    return rvo.get_agent_time_horizon_obst(agent_index);
}

export function rvo_get_agent_velocity(rvo: RVOSimulator, agent_index: i32): Vector2 {
    return rvo.get_agent_velocity(agent_index);
}

export function rvo_get_agents_velocities(rvo: RVOSimulator): StaticArray<f32> {
    return rvo.get_agents_velocities();
}

export function rvo_query_visibility(rvo: RVOSimulator, start_x: f32, start_y: f32, end_x: f32, end_y: f32, radius: f32): bool {
    return rvo.query_visibility(start_x, start_y, end_x, end_y, radius);
}

export function rvo_set_agent_position(rvo: RVOSimulator, agent_index: i32, position_x: f32, position_y: f32): void {
    rvo.set_agent_position(agent_index, position_x, position_y);
}

export function rvo_set_agents_positions(rvo: RVOSimulator, positions: StaticArray<f32>): void {
    rvo.set_agents_positions(positions);
}

export function rvo_set_agent_pref_velocity(rvo: RVOSimulator, agent_index: i32, velocity_x: f32, velocity_y: f32): void {
    rvo.set_agent_pref_velocity(agent_index, velocity_x, velocity_y);
}

export function rvo_set_agents_pref_velocities(rvo: RVOSimulator, velocities: StaticArray<f32>): void {
    rvo.set_agents_pref_velocities(velocities);
}