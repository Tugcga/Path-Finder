import { Navmesh } from "./navmesh";
import { RVOSimulator } from "./rvo_simulator";
import { List } from "./list";
import { Vector2, normalize } from "./vector2";
import { log_message, is_pair_in_list, Pair } from "./utilities";

export class PathFinder{
    private m_navmesh: Navmesh | null;
    private m_groups_count: i32;

    private m_neighbor_dist: f32;
    private m_max_neighbors: i32;
    private m_time_horizon: f32;
    private m_time_horizon_obst: f32;
    private m_agent_radius: f32;
    private m_update_path_find: f32;
    private m_continuous_moving: bool;
    private m_move_agents: bool;
    private m_snap_agents: bool;
    private m_use_normals: bool;
    private m_last_path_find_update: f32;

    private m_is_planar: bool;
    private m_planar_y: f32;

    private m_simulators: StaticArray<RVOSimulator>;  // one simulator per group

    private m_agent_id: i32;
    private m_agents_positions: List<f32>;  // store here 3d-coordinates of the agent, snapped to navigation mesh (if it exists)
    private m_agents_speed: List<f32>;
    private m_agents_activity: List<bool>;
    private m_agents_targets: List<Float32Array>;
    private m_agents_path: List<Float32Array>;
    private m_agents_target_index: List<i32>;
    private m_agents_target_direction: List<f32>;
    private m_agents_group: List<i32>;
    private m_agents_group_id: List<List<i32>>;
    private m_agents_id: List<i32>;

    private m_last_update_time: f32;
    private m_agents_to_delete: List<i32>;

    constructor(vertices: StaticArray<f32> | null, 
                polygons: StaticArray<i32> | null, 
                sizes: StaticArray<i32> | null,
                neighbor_dist: f32 = 1.5,
                max_neighbors: i32 = 5,
                time_horizon: f32 = 1.5,
                time_horizon_obst: f32 = 2.0,
                agent_radius: f32 = 0.2,
                update_path_find: f32 = 1.0,
                continuous_moving: bool = false,
                move_agents: bool = true,
                snap_agents: bool = true,
                use_normals: bool = true) {
        this.m_groups_count = 1;
        if(vertices && polygons && sizes){
            this.m_navmesh = new Navmesh(vertices, polygons, sizes);
        }

        let navmesh_boundary = new List<List<List<Pair<i32>>>>();  // group - component - edge index - vertex pair
        let navmesh = this.m_navmesh;
        if(navmesh){
            // is navmesh is planar
            this.m_is_planar = navmesh.get_is_planar();
            this.m_planar_y = navmesh.get_planar_y();

            // next we should calculate navmesh boundary
            this.m_groups_count = navmesh.get_groups_count();

            for(let g_index = 0, glen = this.m_groups_count; g_index < glen; g_index++){
                let group_polygons = navmesh.get_group_polygons(g_index);
                let all_edges = new List<Pair<i32>>();
                for(let pi = 0, plen = group_polygons.length; pi < plen; pi++){
                    let p = group_polygons[pi];
                    for(let i = 0, len = p.length; i < len; i++){
                        all_edges.push(new Pair<i32>(p[i], p[i + 1 < len ? i + 1 : 0]));
                    }
                }

                let bounday_edges = new List<Pair<i32>>();
                for(let ei = 0, eilen = all_edges.length; ei < eilen; ei++){
                    let e = all_edges[ei];
                    let other_e = new Pair<i32>(e.y(), e.x());
                    if(!is_pair_in_list(other_e, all_edges)){
                        bounday_edges.push(e);
                    }
                }
                let boundary_chains = new List<List<Pair<i32>>>();  // the list of chains, each chain is List<Pair<i32>>
                let current_chain = new List<Pair<i32>>();
                current_chain.push(bounday_edges.pop_last());
                while(bounday_edges.length > 0){
                    let e = current_chain[current_chain.length - 1];
                    let is_find = false;
                    let i = 0;
                    while(!is_find){
                        let f = bounday_edges[i];
                        if(f.x() == e.y()){
                            is_find = true;
                        }
                        else{
                            i++;
                        }
                        if(i >= bounday_edges.length){
                            is_find = true;
                        }
                    }
                    if(i >= bounday_edges.length){
                        //in proper data impossible
                        current_chain = new List<Pair<i32>>();
                        current_chain.push(bounday_edges.pop_last());
                    }
                    else{
                        let f = bounday_edges.pop(i);
                        if(f.y() == current_chain[0].x()){
                            boundary_chains.push(current_chain);
                            if(bounday_edges.length > 0){
                                current_chain = new List<Pair<i32>>();
                                current_chain.push(bounday_edges.pop_last());
                            }
                        }
                        else{
                            current_chain.push(f);
                        }
                    }
                }
                navmesh_boundary.push(boundary_chains);
            }
        }
        else{
            this.m_is_planar = true;
            this.m_planar_y = 0.0;
        }

        this.m_neighbor_dist = neighbor_dist;
        this.m_max_neighbors = max_neighbors;
        this.m_time_horizon = time_horizon;
        this.m_time_horizon_obst = time_horizon_obst;
        this.m_agent_radius = agent_radius;
        this.m_update_path_find = update_path_find;
        this.m_continuous_moving = continuous_moving;
        this.m_move_agents = move_agents;
        this.m_snap_agents = snap_agents;
        this.m_use_normals = use_normals;
        this.m_last_path_find_update = 0.0;

        // create rvo simulators
        this.m_simulators = new StaticArray<RVOSimulator>(this.m_groups_count);
        for(let g = 0, glen = this.m_groups_count; g < glen; g++){
            unchecked(this.m_simulators[g] = new RVOSimulator(this.m_neighbor_dist,
                                                    this.m_max_neighbors,
                                                    this.m_time_horizon,
                                                    this.m_time_horizon_obst,
                                                    this.m_agent_radius,
                                                    0.0));  // set default agent max speed = 0.0, because when we add the agent, then we assign it speed
        }

        const shift_value: f32 = 1.0 * agent_radius;
        if(vertices && navmesh_boundary.length > 0){
            for(let group_index = 0, glen = this.m_groups_count; group_index < glen; group_index++){
                let boundary = navmesh_boundary[group_index];
                for(let ch_index = 0, chlen = boundary.length; ch_index < chlen; ch_index++){
                    let chain = boundary[ch_index];
                    let chain_vertices = new StaticArray<f32>(2*(chain.length + 1));
                    for(let ei = 0, elen = chain.length; ei < elen; ei++){
                        let edge = chain[ei];
                        chain_vertices[2*ei] = vertices[3*(edge.x())];
                        chain_vertices[2*ei + 1] = vertices[3*(edge.x()) + 2];
                    }
                    chain_vertices[2*chain.length] = vertices[3*chain[chain.length - 1].y()];
                    chain_vertices[2*chain.length + 1] = vertices[3*chain[chain.length - 1].y() + 2];

                    // next convert raw positions to shifted edges and pass it to simulator
                    let shifted_chain = new StaticArray<Vector2>(chain_vertices.length / 2);
                    for(let i = 0, len = chain.length + 1; i < len; i++){
                        const pre_index = (i - 1) >= 0 ? i - 1 : chain.length;
                        const post_index = (i + 1) < (chain.length + 1) ? i + 1 : 0;
                        let pre_point = new Vector2(chain_vertices[2*pre_index], chain_vertices[2*pre_index + 1]);
                        let point = new Vector2(chain_vertices[2*i], chain_vertices[2*i + 1]);
                        let post_point = new Vector2(chain_vertices[2*post_index], chain_vertices[2*post_index + 1]);
                        let a1 = point.subtract(pre_point);
                        let a2 = point.subtract(post_point);
                        a1 = normalize(a1);
                        a2 = normalize(a2);
                        let n1 = new Vector2(-a1.y(), a1.x());
                        let n2 = new Vector2(a2.y(), -a2.x());
                        if(Mathf.abs(a1.y()*a2.x() - a1.x()*a2.y()) < 0.0001){
                            shifted_chain[i] = point.add(n1.scale(shift_value));
                        }
                        else{
                            const t = (a2.x()*(post_point.y() + n2.y()*shift_value - pre_point.y() - n1.y()*shift_value) + a2.y()*(pre_point.x() + n1.x()*shift_value - post_point.x() - n2.x()*shift_value)) / (a1.y() * a2.x() - a1.x() * a2.y());
                            let p = n1.scale(shift_value).add(a1.scale(t));
                            shifted_chain[i] = pre_point.add(p);
                        }
                    }

                    unchecked(this.m_simulators[group_index]).add_obstacle(shifted_chain);
                }
                unchecked(this.m_simulators[group_index]).process_obstacles();
            }
        }

        this.m_agent_id = 0;
        this.m_agents_positions = new List<f32>();
        this.m_agents_speed = new List<f32>();
        this.m_agents_activity = new List<bool>();
        this.m_agents_targets = new List<Float32Array>();  // arrays contains 2d-coordinates
        this.m_agents_path = new List<Float32Array>();  // arrays contains 3d-coorfinates
        this.m_agents_target_index = new List<i32>();
        this.m_agents_target_direction = new List<f32>();  // store pairs for each agent
        this.m_agents_group = new List<i32>();
        this.m_agents_group_id = new List<List<i32>>();
        for(let g = 0; g < this.m_groups_count; g++){
            this.m_agents_group_id.push(new List<i32>());
        }
        this.m_agents_id = new List<i32>();
        this.m_agents_to_delete = new List<i32>();
        this.m_last_update_time = 0.0;
    }

    @inline
    get_default_agent_radius(): f32{
        return this.m_agent_radius;
    }

    add_agent(position_x: f32, position_y: f32, position_z: f32,
              radius: f32, speed: f32): i32{
        let is_add = true;
        let x = position_x;
        let y = position_y;
        let z = position_z;
        let add_group = 0;
        let navmesh = this.m_navmesh;
        if(navmesh){
            let node = navmesh.sample_polygon(position_x, position_y, position_z);
            if(node){
                add_group = node.get_group();
                //get closest position on the navmesh
                let sample = navmesh.sample(position_x, position_y, position_z);  // always return 4 values
                if(sample[3] > 0.5){  // valid answer
                    x = sample[0]; y = sample[1]; z = sample[2];
                }
                else{
                    is_add = false;
                }
            }
            else{
                is_add = false;
            }
        }

        if(is_add){
            unchecked(this.m_simulators[add_group]).add_agent_ext(x, z,  // in simulator we use only xz-projections of the position
                                                       0.0, 0.0,
                                                       radius,
                                                       this.m_neighbor_dist,
                                                       this.m_max_neighbors,
                                                       this.m_time_horizon,
                                                       this.m_time_horizon_obst,
                                                       speed);
            this.m_agents_positions.push(x); this.m_agents_positions.push(y); this.m_agents_positions.push(z);
            this.m_agents_speed.push(speed);
            this.m_agents_activity.push(false);
            this.m_agents_targets.push(new Float32Array(0));
            this.m_agents_path.push(new Float32Array(0));
            this.m_agents_target_index.push(0);
            this.m_agents_target_direction.push(0.0);
            this.m_agents_target_direction.push(0.0);
            this.m_agents_group.push(add_group);
            this.m_agents_group_id[add_group].push(this.m_agent_id);

            this.m_agents_id.push(this.m_agent_id);
            this.m_agent_id++;
            return this.m_agent_id - 1;
        }

        return -1;
    }

    @inline
    delete_agent(agent_id: i32): void{
        if(agent_id > -1){
            this.m_agents_to_delete.push(agent_id);
        }
    }

    @inline
    private _to_direction(from_point: Vector2, to_point: Vector2): Vector2{
        let v = to_point.subtract(from_point);
        const l = v.length();
        if(l < 0.00001){
            return new Vector2();
        }
        else{
            return new Vector2(v.x() / l, v.y() / l);
        }
    }

    set_agent_destination(agent_id: i32, position_x: f32, position_y: f32, position_z: f32): bool{
        let is_assign = true;
        let x = position_x;
        let y = position_y;
        let z = position_z;
        let p = this.get_agent_position(agent_id);
        if(p.length > 0){
            // get valid target point on the navmesh
            let navmesh = this.m_navmesh;
            if(navmesh){
                let sample = navmesh.sample(x, y, z);
                if(sample[3] > 0.5){
                    x = sample[0]; y = sample[1]; z = sample[2];
                }
                else{
                    //target point outside navmesh
                    is_assign = false;
                }
            }

            if(is_assign){
                let a_path = this.search_path(p[0], p[1], p[2], x, y, z);
                is_assign = this._set_agent_path(agent_id, a_path);
            }
        }
        else{
            is_assign = false;
        }
        return is_assign;
    }

    private _set_agent_path(agent_id: i32, path: Float32Array): bool{
        // path 0 or at least 6 values
        const agent_index = this._get_agent_inner_index(agent_id);
        if(agent_index > -1){
            if(path.length > 0){
                unchecked(this.m_agents_path[agent_index] = path);
                let targets = new Float32Array(2 * (path.length / 3));
                for(let i = 0, len = targets.length / 2; i < len; i++){
                    unchecked(targets[2*i] = path[3*i]);
                    unchecked(targets[2*i + 1] = path[3*i + 2]);
                }
                unchecked(this.m_agents_targets[agent_index] = targets);
                this.m_agents_target_index[agent_index] = 1;

                let d = this._to_direction(new Vector2(this.m_agents_positions[3*agent_index], this.m_agents_positions[3*agent_index + 2]), new Vector2(targets[2], targets[3]));
                this.m_agents_target_direction[2*agent_index] = d.x();
                this.m_agents_target_direction[2*agent_index + 1] = d.y();

                this.m_agents_activity[agent_index] = true;

                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }

    update(delta_time: f32): void{
        this.m_last_update_time += delta_time;  // increase total time
        const update_path = this.m_last_update_time - this.m_last_path_find_update > this.m_update_path_find;
        if(update_path){
            this.m_last_path_find_update = this.m_last_update_time;  // remember last path time update
        }

        const to_delete_length = this.m_agents_to_delete.length;
        if(to_delete_length > 0){
            //there are agents to delete
            let indexes_in_groups = new StaticArray<List<i32>>(this.m_groups_count);
            let inner_indexes = new StaticArray<i32>(to_delete_length);
            for(let group_index = 0, glen = this.m_groups_count; group_index < glen; group_index++){
                indexes_in_groups[group_index] = new List<i32>();
            }

            for(let ai = 0; ai < to_delete_length; ai++){
                const agent_id = this.m_agents_to_delete[ai];
                const agent_inner_index = this._get_agent_inner_index(agent_id);
                inner_indexes[ai] = agent_inner_index;
                if(agent_inner_index > -1){
                    const group_index = this.m_agents_group[agent_inner_index];
                    const agent_in_group_index = this._get_agent_group_index(agent_id, this.m_agents_group_id[group_index]);
                    indexes_in_groups[group_index].push(agent_in_group_index);
                }
            }

            for(let gi = 0, gilen = this.m_groups_count; gi < gilen; gi++){
                let group_indexes = indexes_in_groups[gi];
                if(group_indexes.length > 0){
                    group_indexes.sort();
                    unchecked(this.m_simulators[gi]).delete_agents(group_indexes);
                }
            }
            inner_indexes.sort();

            for(let i = 0; i < to_delete_length; i++){
                const agent_inner_index = inner_indexes[to_delete_length - 1 - i];
                const agent_id = this.m_agents_id[agent_inner_index];
                const group_index = this.m_agents_group[agent_inner_index];
                const agent_in_group_index = this._get_agent_group_index(agent_id, this.m_agents_group_id[group_index]);
                this.m_agents_target_direction.pop_sequence(2*agent_inner_index, 2);
                this.m_agents_positions.pop_sequence(3*agent_inner_index, 3);
                this.m_agents_target_index.pop(agent_inner_index);
                this.m_agents_targets.pop(agent_inner_index);
                this.m_agents_path.pop(agent_inner_index);
                this.m_agents_activity.pop(agent_inner_index);
                this.m_agents_speed.pop(agent_inner_index);
                this.m_agents_group_id[this.m_agents_group[agent_inner_index]].pop(agent_in_group_index);
                this.m_agents_group.pop(agent_inner_index);
                this.m_agents_id.pop(agent_inner_index);
            }
            this.m_agents_to_delete.reset();
        }

        const activity_count = this.get_active_agents_count();
        if(activity_count > 0){
            let navmesh = this.m_navmesh;
            //next actual update
            for(let agent_inner_index = 0, len = this.m_agents_id.length; agent_inner_index < len; agent_inner_index++){
                const agent_id = this.m_agents_id[agent_inner_index];
                let should_deactivate = false;
                const group_index = this.m_agents_group[agent_inner_index];
                let sim = unchecked(this.m_simulators[group_index]);
                const agent_index = this._get_agent_group_index(agent_id, this.m_agents_group_id[group_index]);

                const spatial_position_x = this.m_agents_positions[3*agent_inner_index];
                const spatial_position_y = this.m_agents_positions[3*agent_inner_index + 1];
                const spatial_position_z = this.m_agents_positions[3*agent_inner_index + 2];
                if(this.m_agents_activity[agent_inner_index]){
                    //this agent should move
                    //get it position
                    let current_position = sim.get_agent_position(agent_index);  // return Vector2
                    const agent_target_index = this.m_agents_target_index[agent_inner_index];
                    const agent_targets_count = unchecked(this.m_agents_targets[agent_inner_index]).length / 2;
                    //and target point
                    let target_x = unchecked(this.m_agents_targets[agent_inner_index][2*agent_target_index]);
                    let target_y = unchecked(this.m_agents_targets[agent_inner_index][2*agent_target_index + 1]);

                    let to_vector = new Vector2(target_x - current_position.x(), target_y - current_position.y());
                    const distance_to_target = to_vector.length();

                    let a_speed = this.m_agents_speed[agent_inner_index];
                    //to calculate proper move speed, we should get normal from the polygon under the agent
                    if(this.m_use_normals && navmesh){
                        let agent_polygon = navmesh.sample_polygon(spatial_position_x, spatial_position_y, spatial_position_z);
                        if(agent_polygon){
                            a_speed *= agent_polygon.get_normal_y();
                        }
                    }

                    if(agent_target_index == agent_targets_count - 1 && distance_to_target < delta_time * a_speed){
                        //we should deactivate the agent when it comes to the final destination
                        //it means that there are no other points in the path and final step can be done in the update call
                        if(distance_to_target < 0.00001){
                            sim.set_agent_pref_velocity(agent_index, 0.0, 0.0);
                            should_deactivate = true;
                        }
                        else{
                            let a_velocity = normalize(to_vector);
                            const last_speed = distance_to_target / delta_time;
                            sim.set_agent_pref_velocity(agent_index, a_velocity.x() * last_speed, a_velocity.y() * last_speed);
                            should_deactivate = true;
                        }
                    }
                    else{
                        let local_dir = this._to_direction(current_position, new Vector2(target_x, target_y));
                        const start_dir_x = this.m_agents_target_direction[2*agent_inner_index];
                        const start_dir_y = this.m_agents_target_direction[2*agent_inner_index + 1];
                        const d = local_dir.x() * start_dir_x + local_dir.y() * start_dir_y;
                        //check, may be we need to swith target point to the next point in the path
                        if(d < 0.0){
                            if(agent_target_index < agent_targets_count - 1){
                                const next_target_x = unchecked(this.m_agents_targets[agent_inner_index][2*agent_target_index + 2]);
                                const next_target_y = unchecked(this.m_agents_targets[agent_inner_index][2*agent_target_index + 3]);
                                const is_next_visible = sim.query_visibility(current_position.x(), current_position.y(), next_target_x, next_target_y, 0.0);
                                if(is_next_visible){
                                    this.m_agents_target_index[agent_inner_index] += 1;
                                    target_x = next_target_x;
                                    target_y = next_target_y;
                                }
                            }
                        }
                    }

                    if(this.m_agents_activity[agent_inner_index]){
                        if(!this.m_continuous_moving && should_deactivate){
                            this.m_agents_activity[agent_inner_index] = false;
                            unchecked(this.m_agents_path[agent_inner_index] = new Float32Array(0));
                        }
                        else{
                            if(update_path && unchecked(this.m_agents_targets[agent_inner_index]).length > 0){
                                let current_path = unchecked(this.m_agents_path[agent_index]);
                                const a_path_count = current_path.length / 3;
                                const target_position_x = current_path[3*(a_path_count - 1)];
                                const target_position_y = current_path[3*(a_path_count - 1) + 1];
                                const target_position_z = current_path[3*(a_path_count - 1) + 2];

                                let a_path = this.search_path(spatial_position_x, spatial_position_y, spatial_position_z,
                                                              target_position_x, target_position_y, target_position_z);
                                this._set_agent_path(agent_id, a_path);
                            }

                            if(!should_deactivate){
                                let to_vector = new Vector2(target_x - current_position.x(), target_y - current_position.y());
                                let a_velocity = normalize(to_vector);
                                sim.set_agent_pref_velocity(agent_index, a_velocity.x() * a_speed, a_velocity.y() * a_speed);
                            }
                        }
                    }
                    else{
                        sim.set_agent_pref_velocity(agent_index, 0.0, 0.0);
                    }
                }
                else{
                    sim.set_agent_pref_velocity(agent_index, 0.0, 0.0);
                }
            }

            for(let i = 0, len = this.m_groups_count; i < len; i++){
                let sim = unchecked(this.m_simulators[i]);
                sim.do_step(delta_time, this.m_move_agents);
            }

            //finally set 3d-positions
            if(this.m_move_agents){
                for(let agent_inner_index = 0, len = this.m_agents_id.length; agent_inner_index < len; agent_inner_index++){
                    const agent_id = this.m_agents_id[agent_inner_index];
                    const group_index = this.m_agents_group[agent_inner_index];
                    let sim = unchecked(this.m_simulators[group_index]);
                    const agent_index = this._get_agent_group_index(agent_id, this.m_agents_group_id[group_index]);
                    let current_position = sim.get_agent_position(agent_index);

                    if(this.m_snap_agents){
                        // current 3d-positions
                        let x = this.m_agents_positions[3*agent_inner_index];
                        let y = this.m_agents_positions[3*agent_inner_index + 1];
                        let z = this.m_agents_positions[3*agent_inner_index + 2];
                        // if there is active navmesh, snap to it
                        if(navmesh){
                            let sample = navmesh.sample(current_position.x(), y, current_position.y());
                            if(sample[3] > 0.5){
                                x = sample[0]; y = sample[1]; z = sample[2];
                            }
                            else{
                                x = current_position.x();
                                z = current_position.y();
                            }
                        }
                        else{
                            x = current_position.x();
                            z = current_position.y();
                        }

                        // set these coordinates as new position
                        this.m_agents_positions[3*agent_inner_index] = x;
                        this.m_agents_positions[3*agent_inner_index + 1] = y;
                        this.m_agents_positions[3*agent_inner_index + 2] = z;
                        //also set position into simulator
                        sim.set_agent_position(agent_index, x, z);
                    }
                    else{
                        //simply copy positions from rvo to 3d-array
                        this.m_agents_positions[3*agent_inner_index] = current_position.x();
                        //calculate y-value
                        if(this.m_is_planar){
                            this.m_agents_positions[3*agent_inner_index + 1] = this.m_planar_y;
                        }
                        else{
                            let agent_path = this.m_agents_path[agent_inner_index];
                            if(agent_path.length > 0){
                                const target_index = this.m_agents_target_index[agent_inner_index];
                                const end_value = agent_path[3*target_index + 1];
                                const start_value = agent_path[3*(target_index - 1) + 1];

                                //we should define parameter t - where current point is between start and end of the path segment
                                //and then interpolate y-position between these two values
                                //get coordinates of the start segment point
                                const start_y = agent_path[3*(target_index - 1) + 1];
                                const end_y = agent_path[3*target_index + 1];
                                if(Mathf.abs(end_y - start_y) < 0.0001){
                                    this.m_agents_positions[3*agent_inner_index + 1] = end_y;
                                }
                                else{
                                    const start_x = agent_path[3*(target_index - 1)];
                                    const start_z = agent_path[3*(target_index - 1) + 2];
                                    //end point of the segment
                                    const end_x = agent_path[3*target_index];
                                    const end_z = agent_path[3*target_index + 2];
                                    //calculate coordinates to the point and of the segment
                                    const to_point_x = current_position.x() - start_x;
                                    const to_point_z = current_position.y() - start_z;
                                    const segment_x = end_x - start_x;
                                    const segment_z = end_z - start_z;
                                    //calculate dot product between these two vectors
                                    const d = to_point_x*segment_x + to_point_z*segment_z;
                                    //also square length of the segment
                                    const segment_sql = segment_x*segment_x + segment_z*segment_z;
                                    //finall parameter
                                    if(segment_sql < 0.0001){
                                        this.m_agents_positions[3*agent_inner_index + 1] = end_y;
                                    }
                                    else{
                                        const t_param = d / segment_sql;
                                        if(t_param < 0.0){
                                            this.m_agents_positions[3*agent_inner_index + 1] = start_y;
                                        }
                                        else if(t_param > 1.0){
                                            this.m_agents_positions[3*agent_inner_index + 1] = end_y;
                                        }
                                        else{
                                            this.m_agents_positions[3*agent_inner_index + 1] = end_y * t_param + start_y * (1 - t_param);
                                        }
                                    }
                                }
                            }
                        }
                        this.m_agents_positions[3*agent_inner_index + 2] = current_position.y();
                    }
                }
            }
        }
    }

    @inline
    get_all_agents_positions(): Float32Array{
        let to_return = new Float32Array(this.m_agents_positions.length);
        for(let i = 0, len = this.m_agents_positions.length; i < len; i++){
            to_return[i] = this.m_agents_positions[i];
        }
        return to_return;
    }

    @inline
    get_all_agents_velocities(): Float32Array{
        let to_return = new Float32Array(2 * this.m_agents_id.length);
        for(let agent_inner_index = 0, len = this.m_agents_id.length; agent_inner_index <len; agent_inner_index++){
            const agent_group = this.m_agents_group[agent_inner_index];
            const agent_id = this.m_agents_id[agent_inner_index];
            let sim = unchecked(this.m_simulators[agent_group]);
            const agent_index = this._get_agent_group_index(agent_id, this.m_agents_group_id[agent_group]);
            let velocity = sim.get_agent_velocity(agent_index);
            to_return[2*agent_inner_index] = velocity.x();
            to_return[2*agent_inner_index + 1] = velocity.y();
        }
        return to_return;
    }

    @inline
    private _get_agent_inner_index(agent_id: i32): i32{
        for(let i = 0, len = this.m_agents_id.length; i < len; i++){
            const v = this.m_agents_id[i];
            if(v == agent_id){
                return i;
            }
        }
        return -1;
    }

    @inline
    get_agent_path(agent_id: i32): Float32Array{
        const agent_index = this._get_agent_inner_index(agent_id);
        if(agent_index > -1){
            return unchecked(this.m_agents_path[agent_index]);
        }
        else{
            return new Float32Array(0);
        }
    }

    @inline
    get_all_agents_activities(): StaticArray<bool>{
        let to_return = new StaticArray<bool>(this.m_agents_activity.length);
        for(let i = 0, len = to_return.length; i < len; i++){
            to_return[i] = this.m_agents_activity[i];
        }
        return to_return;
    }

    @inline
    get_agent_activity(agent_id: i32): bool{
        const agent_inner_index = this._get_agent_inner_index(agent_id);
        if(agent_inner_index > -1){
            return this.m_agents_activity[agent_inner_index];
        }
        else{
            return false;
        }
    }

    @inline
    private _get_agent_group_index(agent_id: i32, group_ids: List<i32>): i32{
        for(let i = 0, len = group_ids.length; i < len; i++){
            const v = group_ids[i];
            if(v == agent_id){
                return i;
            }
        }
        return -1;
    }

    @inline
    get_agent_velocity(agent_id: i32): Float32Array{
        const agent_inner_index = this._get_agent_inner_index(agent_id);
        if(agent_inner_index > -1){
            const agent_group = this.m_agents_group[agent_inner_index];
            let sim = unchecked(this.m_simulators[agent_group]);
            const agent_index = this._get_agent_group_index(agent_id, this.m_agents_group_id[agent_group]);
            let velocity = sim.get_agent_velocity(agent_index);
            let to_return = new Float32Array(2);
            to_return[0] = velocity.x();
            to_return[1] = velocity.y();

            return to_return;
        }
        else{
            return new Float32Array(0);
        }
    }

    @inline
    get_agent_position(agent_id: i32): Float32Array{
        const agent_inner_index = this._get_agent_inner_index(agent_id);
        if(agent_inner_index > -1){
            let to_return = new Float32Array(3);
            to_return[0] = this.m_agents_positions[3*agent_inner_index];
            to_return[1] = this.m_agents_positions[3*agent_inner_index + 1];
            to_return[2] = this.m_agents_positions[3*agent_inner_index + 2];
            return to_return;
        }
        else{
            return new Float32Array(0);
        }
    }

    @inline
    get_agents_count(): i32{
        return this.m_agents_id.length;
    }

    @inline
    get_agents_id(): Int32Array{
        let to_return = new Int32Array(this.m_agents_id.length);
        for(let i = 0, len = to_return.length; i < len; i++){
            to_return[i] = this.m_agents_id[i];
        }
        return to_return;
    }

    @inline
    get_active_agents_count(): i32{
        let count = 0;
        for(let i = 0, len = this.m_agents_id.length; i < len; i++){
            if(this.m_agents_activity[i]){
                count++;
            }
        }
        return count;
    }

    @inline
    search_path(s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): Float32Array {
        let nm = this.m_navmesh;
        if(nm){
            let path = nm.search_path(s_x, s_y, s_z, e_x, e_y, e_z);
            const length = path.length;
            if(length == 0 || length >= 6){
                return path;
            }
            else{
                //return only three value, so, add the target point into the output
                let to_return = new Float32Array(6);
                to_return[0] = path[0]; to_return[1] = path[1]; to_return[2] = path[2]; 
                to_return[3] = e_x; to_return[4] = e_y; to_return[5] = e_z; 
                return to_return;
            }
        }
        else{
            let to_return = new Float32Array(6);
            to_return[0] = s_x; to_return[1] = s_y; to_return[2] = s_z;
            to_return[3] = e_x; to_return[4] = e_y; to_return[5] = e_z;
            return to_return;
        }
    }

    @inline
    sample(x: f32, y: f32, z: f32): Float32Array{
        // return point on the navmesh close to the input position
        // return 4 values: x, y, z and valid key (0.0 - invald input, 1.0 - valid output)
        let navmesh = this.m_navmesh;
        if(navmesh){
            return navmesh.sample(x, y, z);
        }
        else{
            let to_return = new Float32Array(4);
            to_return[0] = x; to_return[1] = y; to_return[2] = z; to_return[3] = 1.0;
            return to_return;
        }
    }

    @inline
    get_neighbor_dist(): f32{
        return this.m_neighbor_dist;
    }

    @inline
    get_max_neighbors(): i32{
        return this.m_max_neighbors;
    }

    @inline
    get_time_horizon(): f32{
        return this.m_time_horizon;
    }

    @inline
    get_time_horizon_obst(): f32{
        return this.m_time_horizon_obst;
    }

    @inline
    get_update_path_find(): f32{
        return this.m_update_path_find;
    }

    @inline
    set_update_path_find(value: f32): void{
        this.m_update_path_find = value;
    }

    @inline
    get_continuous_moving(): bool{
        return this.m_continuous_moving;
    }

    @inline
    set_continuous_moving(value: bool): void{
        this.m_continuous_moving = value;
    }

    @inline
    get_move_agents(): bool{
        return this.m_move_agents;
    }

    @inline
    set_move_agents(value: bool): void{
        this.m_move_agents = value;
    }

    @inline
    get_snap_agents(): bool{
        return this.m_snap_agents;
    }

    @inline
    set_snap_agents(value: bool): void{
        this.m_snap_agents = value;
    }

    @inline
    get_use_normals(): bool{
        return this.m_use_normals;
    }

    @inline
    set_use_normals(value: bool): void{
        this.m_use_normals = value;
    }

    @inline
    get_rvo_simulator(group: i32): RVOSimulator | null{
        if(group < this.m_groups_count){
            return unchecked(this.m_simulators[group]);
        }
        else{
            return null;
        }
    }

    @inline
    get_navmesh(): Navmesh | null{
        return this.m_navmesh;
    }
}