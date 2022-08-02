import { KDTree } from "./rvo_kd_tree";
import { Obstacle } from "./rvo_obstacle";
import { Agent } from "./rvo_agent";
import { Vector2 } from "../common/vector2";
import { List } from "../common/list";
import { log_message, is_in_array } from "../common/utilities";

export class RVOSimulator{
    private m_agents: List<Agent>;
    private m_agents_count: i32;
    private m_kd_tree: KDTree;
    private m_obstacles: List<Obstacle>;

    private m_neighbor_dist: f32;
    private m_max_neighbors: i32;
    private m_time_horizon: f32;
    private m_time_horizon_obst: f32;
    private m_radius: f32;
    private m_max_speed: f32;

    private m_force_update_agents: bool;

    constructor(neighbor_dist: f32,
                max_neighbors: i32,
                time_horizon: f32,
                time_horizon_obst: f32,
                radius: f32,
                max_speed: f32) {
        this.m_neighbor_dist = neighbor_dist;
        this.m_max_neighbors = max_neighbors;
        this.m_time_horizon = time_horizon;
        this.m_time_horizon_obst = time_horizon_obst;
        this.m_radius = radius;
        this.m_max_speed = max_speed;
        this.m_kd_tree = new KDTree();
        this.m_agents = new List<Agent>();
        this.m_agents_count = 0;
        this.m_obstacles = new List<Obstacle>();

        this.m_force_update_agents = true;
        this.m_kd_tree.set_simulator(this);
    }

    @inline
    add_agent(position_x: f32, position_y: f32): i32{
        return this.add_agent_ext(position_x, position_y, 
                                  0.0, 0.0,
                                  this.m_radius,
                                  this.m_neighbor_dist,
                                  this.m_max_neighbors,
                                  this.m_time_horizon,
                                  this.m_time_horizon_obst,
                                  this.m_max_speed);
    }

    @inline
    add_agent_ext(position_x: f32, position_y: f32,
                  velocity_x: f32, velocity_y: f32,
                  radius: f32,
                  neighbor_dist: f32,
                  max_neighbors: i32,
                  time_horizon: f32,
                  time_horizon_obst: f32,
                  max_speed: f32): i32{
        let agent = new Agent(this, this.m_agents.length,
                              new Vector2(position_x, position_y),
                              new Vector2(velocity_x, velocity_y),
                              neighbor_dist,
                              max_neighbors,
                              time_horizon,
                              time_horizon_obst,
                              radius,
                              max_speed);
        this.m_agents.push(agent);
        this.m_agents_count++;
        this.m_force_update_agents = true;
        return this.m_agents.length - 1;
    }

    @inline
    delete_agent(agent_index: i32): void{
        let to_delete = new List<i32>();
        to_delete.push(agent_index);
        this.delete_agents(to_delete);
    }

    @inline
    delete_agents(agent_indexes: List<i32>): void{
        var old_index: i32 = 0;
        const agents_count = this.m_agents.length;
        var new_agents = new List<Agent>(agents_count - agent_indexes.length);
        while(old_index < agents_count){
            if(!is_in_array(old_index, agent_indexes)){
                new_agents.push(this.m_agents[old_index]);
            }
            old_index++;
        }
        this.m_agents = new_agents;
        this.m_agents_count = this.m_agents.length;
        this.m_force_update_agents = true;
    }

    @inline
    get_agent(index: i32): Agent{
        return this.m_agents[index];
    }

    @inline
    get_agents_count(): i32{
        return this.m_agents_count;
    }

    //the same as add_obstacle, but input is plain float array
    add_obstacle_array(vertices: StaticArray<f32>): i32{
        let vec2_vertices = new StaticArray<Vector2>(vertices.length / 2);
        for(let  i = 0, len = vec2_vertices.length; i < len; i++){
            unchecked(vec2_vertices[i]).set_values(unchecked(vertices[2*i]), unchecked(vertices[2*i + 1]));
        }
        return this.add_obstacle(vec2_vertices);
    }

    add_obstacle(vertices: StaticArray<Vector2>): i32{
        const vertices_length = vertices.length;
        if (vertices_length < 2) {
            return -1;
        }

        const obstacle_no: i32 = this.m_obstacles.length;

        for (let i = 0; i < vertices_length; i++) {
            var obstacle = new Obstacle();
            obstacle.set_point(unchecked(vertices[i]));

            if (i != 0) {
                obstacle.set_prev_obstacle(this.m_obstacles[this.m_obstacles.length - 1]);
                let po = obstacle.get_prev_obstacle();
                if(po){
                    po.set_next_obstacle(obstacle);
                }
            }

            if (i == vertices_length - 1) {
                obstacle.set_next_obstacle(this.m_obstacles[obstacle_no]);
                let no = obstacle.get_next_obstacle();
                if(no){
                    no.set_prev_obstacle(obstacle);
                }
            }

            const p1 = unchecked(vertices[(i == vertices_length - 1 ? 0 : i + 1)]);
            const p2 = unchecked(vertices[i]);
            obstacle.set_unit_dir_point(p1.x(), p1.y(), p2.x(), p2.y());

            if (vertices_length == 2) {
                obstacle.set_is_convex_value(true);
            }
            else {
                const p3 = unchecked(vertices[(i == 0 ? vertices_length - 1 : i - 1)]);
                obstacle.set_is_convex(p3.x(), p3.y(), p2.x(), p2.y(), p1.x(), p1.y());
            }
            obstacle.set_id(this.m_obstacles.length);
            this.m_obstacles.push(obstacle);
        }

        return obstacle_no;
    }

    @inline
    get_obstacles_count(): i32{
        return this.m_obstacles.length;
    }

    @inline
    get_obstacle(index: i32): Obstacle{
        return this.m_obstacles[index];
    }

    @inline
    compute_agent_obstacles_neighbors(agent: Agent, range_square: f32): void{
        this.m_kd_tree.compute_obstacle_neighbors(agent, range_square);
    }

    @inline
    compute_agent_agents_neighbors(agent: Agent, range_square: f32): void{
        this.m_kd_tree.compute_agent_neighbors(agent, range_square);
    }

    @inline
    add_obstacle_object(obstacle: Obstacle): void{
        this.m_obstacles.push(obstacle);
    }

    do_step_default(delta_time: f32): void{
        this.do_step(delta_time, true);
    }    

    do_step(delta_time: f32, move_agents: bool = true): void{
        this.m_kd_tree.build_agent_tree(this.m_force_update_agents);
        this.m_force_update_agents = false;
        for(let i = 0, len = this.m_agents_count; i < len; i++){
            let agent = this.m_agents[i];
            agent.compute_neighbors();
            agent.compute_new_velocity(delta_time);
        }

        for(let i = 0, len = this.m_agents_count; i < len; i++){
            this.m_agents[i].update(delta_time, move_agents);
        }
    }

    @inline
    get_agent_agent_neighbor(agent_index: i32, neighbor_index: i32): i32{
        return this.m_agents[agent_index].get_agent_agent_neighbor(neighbor_index).get_id();
    }

    @inline
    get_agent_max_neighbors(agent_index: i32): i32{
        return this.m_agents[agent_index].get_max_neighbors();
    }

    @inline
    get_agent_max_speed(agent_index: i32): f32{
        return this.m_agents[agent_index].get_max_speed();
    }

    @inline
    get_agent_neighbor_dist(agent_index: i32): f32{
        return this.m_agents[agent_index].get_neighbor_distance();
    }

    @inline
    get_agent_num_agent_neighbors(agent_index: i32): i32{
        return this.m_agents[agent_index].get_agent_neighbors_count();
    }

    @inline
    get_agent_num_obstacle_neighbors(agent_index: i32): i32{
        return this.m_agents[agent_index].get_num_obstacle_neighbors();
    }

    @inline
    get_agent_obstacle_neighbor(agent_index: i32, neighbor_index: i32): i32{
        return this.m_agents[agent_index].get_obstacle_neighbor(neighbor_index).get_id();
    }

    @inline
    get_agent_position(agent_index: i32): Vector2{
        return this.m_agents[agent_index].get_position();
    }

    //return one plain float array with 2d-positions of all agents in the simulator
    @inline
    get_agents_positions(): StaticArray<f32> {
        let to_return = new StaticArray<f32>(2 * this.m_agents_count);
        for(let i = 0, len = this.m_agents_count; i < len; i++){
            let p = this.get_agent_position(i);
            unchecked(to_return[2*i] = p.x());
            unchecked(to_return[2*i + 1] = p.y());
        }
        return to_return;
    }

    @inline
    get_agent_pref_velocity(agent_index: i32): Vector2{
        return this.m_agents[agent_index].get_pref_velocity();
    }

    @inline
    get_agent_radius(agent_index: i32): f32{
        return this.m_agents[agent_index].get_radius();
    }

    @inline
    get_agent_time_horizon(agent_index: i32): f32{
        return this.m_agents[agent_index].get_time_horizon();
    }

    @inline
    get_agent_time_horizon_obst(agent_index: i32): f32{
        return this.m_agents[agent_index].get_time_horizon_obst();
    }

    @inline
    get_agent_velocity(agent_index: i32): Vector2{
        return this.m_agents[agent_index].get_velocity();
    }

    //return one plain float array with velocities of all agents
    @inline
    get_agents_velocities(): StaticArray<f32> {
        let to_return = new StaticArray<f32>(2 * this.m_agents_count);
        for(let i = 0, len = this.m_agents_count; i < len; i++){
            let v = this.get_agent_velocity(i);
            unchecked(to_return[2*i] = v.x());
            unchecked(to_return[2*i + 1] = v.y());
        }
        return to_return;
    }

    @inline
    get_obstacle_vertex(vertex_index: i32): Vector2{
        return this.m_obstacles[vertex_index].get_point();
    }

    @inline
    get_next_obstacle_vertex_index(vertex_index: i32): i32{
        let obst = this.m_obstacles[vertex_index].get_next_obstacle();
        if(obst){
            return obst.get_id();
        }
        else{
            return -1;
        }
    }

    @inline
    get_prev_obstacle_vertex_index(vertex_index: i32): i32{
        let obst = this.m_obstacles[vertex_index].get_prev_obstacle();
        if(obst){
            return obst.get_id();
        }
        else{
            return -1;
        }
    }

    @inline
    process_obstacles(): void{
        this.m_kd_tree.build_obstacle_tree();
    }

    @inline
    query_visibility(start_x: f32, start_y: f32,
                     end_x: f32, end_y: f32,
                     radius: f32): bool{
        return this.m_kd_tree.query_visibility(new Vector2(start_x, start_y), new Vector2(end_x, end_y), radius);
    }

    @inline
    set_agent_max_neighbors(agent_index: i32, max_neighbors: i32): void{
        this.m_agents[agent_index].set_max_neighbors(max_neighbors);
    }

    @inline
    set_agent_max_speed(agent_index: i32, max_speed: f32): void{
        this.m_agents[agent_index].set_max_speed(max_speed);
    }

    @inline
    set_agent_neighbor_dist(agent_index: i32, neighbor_dist: f32): void{
        this.m_agents[agent_index].set_neighbor_distance(neighbor_dist);
    }

    @inline
    set_agent_position(agent_index: i32, position_x: f32, position_y: f32): void{
        this.m_agents[agent_index].set_position(new Vector2(position_x, position_y));
    }

    //input is array with 2d-positions of all agents
    @inline
    set_agents_positions(positions: StaticArray<f32>): void{
        const input_positions = positions.length / 2;
        const count = input_positions < this.m_agents_count ? input_positions : this.m_agents_count;
        for(let i = 0; i < count; i++){
            this.set_agent_position(i, unchecked(positions[2*i]), unchecked(positions[2*i + 1]));
        }
    }

    @inline
    set_agent_pref_velocity(agent_index: i32, velocity_x: f32, velocity_y: f32): void{
        this.m_agents[agent_index].set_pref_velocity(new Vector2(velocity_x, velocity_y));
    }

    //input is array with prefered 2d-velocities for all agents
    @inline
    set_agents_pref_velocities(velocities: StaticArray<f32>): void{
        const input_velocities = velocities.length / 2;
        const count = input_velocities < this.m_agents_count ? input_velocities : this.m_agents_count;
        for(let i = 0; i < count; i++){
            this.set_agent_pref_velocity(i, unchecked(velocities[2*i]), unchecked(velocities[2*i + 1]));
        }
        //for other agents set zero
        for(let i = 0; i < this.m_agents_count - count; i++){
            this.set_agent_pref_velocity(count + i, 0.0, 0.0);
        }
    }

    @inline
    set_agent_radius(agent_index: i32, radius: f32): void{
        this.m_agents[agent_index].set_radius(radius);
    }

    @inline
    set_agent_time_horizon(agent_index: i32, time_horizon: f32): void{
        this.m_agents[agent_index].set_time_horizon(time_horizon);
    }

    @inline
    set_agent_time_horizon_obst(agent_index: i32, time_horizon_obst: f32): void{
        this.m_agents[agent_index].set_time_horizon_obst(time_horizon_obst);
    }

    @inline
    set_agent_velocity(agent_index: i32, velocity_x: f32, velocity_y: f32): void{
        this.m_agents[agent_index].set_velocity(new Vector2(velocity_x, velocity_y));
    }
}
