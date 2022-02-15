import { Obstacle } from "./rvo_obstacle";
import { Agent } from "./rvo_agent";
import { RVOSimulator } from "./rvo_simulator";
import { log_message, left_of_points, sqr, RVO_EPSILON } from "../common/utilities";
import { Vector2, det, abs_sq } from "../common/vector2";

class AgentTreeNode{
    m_begin: i32;
    m_end: i32;
    m_left: i32;
    m_right: i32;

    m_max_x: f32;
    m_max_y: f32;
    m_min_x: f32;
    m_min_y: f32;
}

class ObstacleTreeNode{
    m_left: ObstacleTreeNode | null;
    m_right: ObstacleTreeNode | null;
    m_obstacle: Obstacle = new Obstacle();

    @inline
    set_obstacle(obstacle: Obstacle): void{
        this.m_obstacle = obstacle;
    }

    @inline
    get_obstacle(): Obstacle{
        return this.m_obstacle;
    }

    @inline
    set_left(node: ObstacleTreeNode | null): void{
        this.m_left = node;
    }

    @inline
    get_left(): ObstacleTreeNode | null{
        return this.m_left;
    }

    @inline
    set_right(node: ObstacleTreeNode | null): void{
        this.m_right = node;

    }

    @inline
    get_right(): ObstacleTreeNode | null{
        return this.m_right;
    }

    to_string(): string{
        let left = this.m_left;
        let right = this.m_right;
        if(left && right){
            return "<" + this.m_obstacle.get_id().toString() + ">:[" + left.to_string() + "-" + right.to_string() + "]";
        }
        else{
            if(left){
                return "<" + this.m_obstacle.get_id().toString() + ">:[" + left.to_string() + "-null]";
            }
            else if(right){
                return "<" + this.m_obstacle.get_id().toString() + ">:[null-" + right.to_string() + "]";
            }
            else{
                return "<" + this.m_obstacle.get_id().toString() + ">:[null-null]";
            }
        }
    }
}

export class KDTree{
    m_sim: RVOSimulator | null;
    m_agents: StaticArray<Agent>;
    m_agent_tree: StaticArray<AgentTreeNode>;
    m_obstacle_tree: ObstacleTreeNode | null;

    MAX_LEAF_SIZE: i32 = 10;

    constructor() {
        this.m_agents = new StaticArray<Agent>(0);
        this.m_agent_tree = new StaticArray<AgentTreeNode>(0);
    }

    set_simulator(sim: RVOSimulator): void{
        this.m_sim = sim;
    }

    build_agent_tree(force_update: bool): void{
        if(force_update){
            let sim = this.m_sim;
            if(sim){
                const agents_count = sim.get_agents_count();
                this.m_agents = new StaticArray<Agent>(agents_count);
                for(let i = 0; i < agents_count; i++){
                    this.m_agents[i] = sim.get_agent(i);
                }
                this.m_agent_tree = new StaticArray<AgentTreeNode>(agents_count > 0 ?  2 * agents_count - 1 : 0);
                for(let i = 0, len = this.m_agent_tree.length; i < len; i++){
                    this.m_agent_tree[i] = new AgentTreeNode();
                }
            }
        }
        if(this.m_agents.length > 0){
            this.build_agent_tree_recirsive(0, this.m_agents.length, 0);
        }
    }

    build_agent_tree_recirsive(begin: i32, end: i32, node: i32): void{
        let tree_node = this.m_agent_tree[node];
        tree_node.m_begin = begin;
        tree_node.m_end = end;
        let begin_position = this.m_agents[begin].get_position();
        tree_node.m_min_x = begin_position.x();
        tree_node.m_max_x = begin_position.x();
        tree_node.m_min_y = begin_position.y();
        tree_node.m_max_y = begin_position.y();

        for (let i = begin + 1; i < end; i++) {
            let i_position = this.m_agents[i].get_position();
            tree_node.m_max_x = Mathf.max(tree_node.m_max_x, i_position.x());
            tree_node.m_min_x = Mathf.min(tree_node.m_min_x, i_position.x());
            tree_node.m_max_y = Mathf.max(tree_node.m_max_y, i_position.y());
            tree_node.m_min_y = Mathf.min(tree_node.m_min_y, i_position.y());
        }

        if (end - begin > this.MAX_LEAF_SIZE) {
            // No leaf node.
            const is_vertical: bool = (tree_node.m_max_x - tree_node.m_min_x > tree_node.m_max_y - tree_node.m_min_y);
            const split_value: f32 = (is_vertical ? 0.5 * (tree_node.m_max_x + tree_node.m_min_x) : 0.5 * (tree_node.m_max_y + tree_node.m_min_y));

            var left = begin;
            var right = end;

            while (left < right) {
                while (left < right && (is_vertical ? this.m_agents[left].get_position().x() : this.m_agents[left].get_position().y()) < split_value) {
                    ++left;
                }

                while (right > left && (is_vertical ? this.m_agents[right - 1].get_position().x() : this.m_agents[right - 1].get_position().y()) >= split_value) {
                    --right;
                }

                if (left < right) {
                    let c = this.m_agents[left];
                    this.m_agents[left] = this.m_agents[right - 1];
                    this.m_agents[right - 1] = c;
                    ++left;
                    --right;
                }
            }

            if (left == begin) {
                ++left;
                ++right;
            }

            tree_node.m_left = node + 1;
            tree_node.m_right = node + 2 * (left - begin);

            this.build_agent_tree_recirsive(begin, left, tree_node.m_left);
            this.build_agent_tree_recirsive(left, end, tree_node.m_right);
        }
    }

    build_obstacle_tree(): void{
        let sim = this.m_sim;
        if(sim){
            const obstacles_count = sim.get_obstacles_count();
            var obstacles = new StaticArray<Obstacle>(obstacles_count);
            for(let i = 0; i < obstacles_count; i++){
                obstacles[i] = sim.get_obstacle(i);
            }
            this.m_obstacle_tree = this.build_obstacle_tree_recursive(obstacles);
        }
    }

    build_obstacle_tree_recursive(obstacles: StaticArray<Obstacle>): ObstacleTreeNode | null{
        if (obstacles.length == 0) {
            return null;
        }
        else {
            const obstacles_count = obstacles.length;
            var node = new ObstacleTreeNode();

            let optimal_split = 0;
            let min_left = obstacles_count;
            let min_right = obstacles_count;

            for (let i = 0; i < obstacles_count; i++) {
                let left_size = 0;
                let right_size = 0;

                let obstacleI1 = obstacles[i];
                let obstacleI2 = obstacleI1.get_next_obstacle();
                if(obstacleI2){
                    // Compute optimal split node.
                    for (let j = 0; j < obstacles_count; j++) {
                        if (i == j) {
                            continue;
                        }

                        let obstacleJ1 = obstacles[j];
                        let obstacleJ2 = obstacleJ1.get_next_obstacle();

                        if(obstacleJ2){
                            let pi1 = obstacleI1.get_point();
                            let pi2 = obstacleI2.get_point();
                            let pj1 = obstacleJ1.get_point();
                            let pj2 = obstacleJ2.get_point();
                            const j1LeftOfI = left_of_points(pi1, pi2, pj1);
                            const j2LeftOfI = left_of_points(pi1, pi2, pj2);

                            if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
                                left_size++;
                            }
                            else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
                                right_size++;
                            }
                            else {
                                left_size++;
                                right_size++;
                            }

                            const a1 = left_size > right_size ? left_size : right_size;
                            const b1 = left_size < right_size ? left_size : right_size;

                            const a2 = min_left > min_right ? min_left : min_right;
                            const b2 = min_left < min_right ? min_left : min_right;
                            if(a1 > a2 || (a1 == a2 && b1 >= b2)){
                                break;
                            }
                        }
                    }

                    const a1 = left_size > right_size ? left_size : right_size;
                    const b1 = left_size < right_size ? left_size : right_size;

                    const a2 = min_left > min_right ? min_left : min_right;
                    const b2 = min_left < min_right ? min_left : min_right;

                    if(a1 < a2 || (a1 == a2 && b1 < b2)){
                        min_left = left_size;
                        min_right = right_size;
                        optimal_split = i;
                    }
                }
            }

            // Build split node.
            var left_obstacles = new StaticArray<Obstacle>(min_left);
            var right_obstacles = new StaticArray<Obstacle>(min_right);

            let left_counter = 0;
            let right_counter = 0;
            const i = optimal_split;

            let obstacleI1 = obstacles[i];
            let obstacleI2 = obstacleI1.get_next_obstacle();
            if(obstacleI2){
                for (let j = 0; j < obstacles_count; j++) {
                    if (i == j) {
                        continue;
                    }

                    let obstacleJ1 = obstacles[j];
                    let obstacleJ2 = obstacleJ1.get_next_obstacle();
                    if(obstacleJ2){
                        let pi1 = obstacleI1.get_point();
                        let pi2 = obstacleI2.get_point();
                        let pj1 = obstacleJ1.get_point();
                        let pj2 = obstacleJ2.get_point();
                        const j1LeftOfI = left_of_points(pi1, pi2, pj1);
                        const j2LeftOfI = left_of_points(pi1, pi2, pj2);

                        if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
                            left_obstacles[left_counter] = obstacles[j];
                            left_counter++;
                        }
                        else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
                            right_obstacles[right_counter] = obstacles[j];
                            right_counter++;
                        }
                        else {
                            // Split obstacle j
                            let v1 = pi2.subtract(pi1);
                            let v2 = pj1.subtract(pi1);
                            let v3 = pj1.subtract(pj2);
                            const t: f32 = det(v1, v2) / det(v1, v3);
                            let v4 = pj2.subtract(pj1);
                            const split_point = pj1.add(v4.scale(t));

                            var new_obstacle = new Obstacle();
                            new_obstacle.set_point(split_point);
                            new_obstacle.set_prev_obstacle(obstacleJ1);
                            new_obstacle.set_next_obstacle(obstacleJ2);
                            new_obstacle.set_is_convex_value(true);
                            new_obstacle.set_unit_dir(obstacleJ1.get_unit_dir());
                            let sim = this.m_sim;
                            if(sim){
                                new_obstacle.set_id(sim.get_obstacles_count());
                                sim.add_obstacle_object(new_obstacle);
                                obstacleJ1.set_next_obstacle(new_obstacle);
                                obstacleJ2.set_prev_obstacle(new_obstacle);

                                if (j1LeftOfI > 0.0) {
                                    left_obstacles[left_counter] = obstacleJ1;
                                    left_counter++;
                                    right_obstacles[right_counter] = new_obstacle;
                                    right_counter++;
                                }
                                else {
                                    right_obstacles[right_counter] = obstacleJ1;
                                    right_counter++;
                                    left_obstacles[left_counter] = new_obstacle;
                                    left_counter++;
                                }
                            }
                        }
                    }
                }
            }
            node.set_obstacle(obstacleI1);
            node.set_left(this.build_obstacle_tree_recursive(left_obstacles));
            node.set_right(this.build_obstacle_tree_recursive(right_obstacles));
            return node;
        }
    }

    compute_agent_neighbors(agent: Agent, range_sq: f32): void{
        range_sq = this.query_agent_tree_recursive(agent, range_sq, 0);
    }

    compute_obstacle_neighbors(agent: Agent, range_sq: f32): void{
        this.query_obstacle_tree_recursive(agent, range_sq, this.m_obstacle_tree);
    }

    query_agent_tree_recursive(agent: Agent, range_sq: f32, node: i32): f32{
        var tree_node = this.m_agent_tree[node];
        if (tree_node.m_end - tree_node.m_begin <= this.MAX_LEAF_SIZE) {
            for (let i = tree_node.m_begin; i < tree_node.m_end; i++) {
                range_sq = agent.insert_agent_neighbor(this.m_agents[i], range_sq);
            }
        }
        else {
            var pos = agent.get_position();
            const x = pos.x();
            const y = pos.y();
            const distSqLeft: f32 = sqr(Mathf.max(0.0, this.m_agent_tree[tree_node.m_left].m_min_x - x)) + sqr(Mathf.max(0.0, x - this.m_agent_tree[tree_node.m_left].m_max_x)) + sqr(Mathf.max(0.0, this.m_agent_tree[tree_node.m_left].m_min_y - y)) + sqr(Mathf.max(0.0, y - this.m_agent_tree[tree_node.m_left].m_max_y));
            const distSqRight: f32 = sqr(Mathf.max(0.0, this.m_agent_tree[tree_node.m_right].m_min_x - x)) + sqr(Mathf.max(0.0, x - this.m_agent_tree[tree_node.m_right].m_max_x)) + sqr(Mathf.max(0.0, this.m_agent_tree[tree_node.m_right].m_min_y - y)) + sqr(Mathf.max(0.0, y - this.m_agent_tree[tree_node.m_right].m_max_y));

            if (distSqLeft < distSqRight) {
                if (distSqLeft < range_sq) {
                    range_sq = this.query_agent_tree_recursive(agent, range_sq, tree_node.m_left);

                    if (distSqRight < range_sq) {
                        range_sq = this.query_agent_tree_recursive(agent, range_sq, tree_node.m_right);
                    }
                }
            }
            else {
                if (distSqRight < range_sq) {
                    range_sq = this.query_agent_tree_recursive(agent, range_sq, tree_node.m_right);

                    if (distSqLeft < range_sq) {
                        range_sq = this.query_agent_tree_recursive(agent, range_sq, tree_node.m_left);
                    }
                }
            }

        }
        return range_sq;
    }

    query_obstacle_tree_recursive(agent: Agent, range_sq: f32, node: ObstacleTreeNode | null): void{
        if(node){
            let obstacle1 = node.get_obstacle();
            let obstacle2 = obstacle1.get_next_obstacle();
            if(obstacle2){
                let p1 = obstacle1.get_point();
                let p2 = obstacle2.get_point();
                const agentLeftOfLine: f32 = left_of_points(p1, p2, agent.get_position());
                this.query_obstacle_tree_recursive(agent, range_sq, (agentLeftOfLine >= 0.0 ? node.get_left() : node.get_right()));

                let v = p2.subtract(p1);
                const distSqLine: f32 = sqr(agentLeftOfLine) / abs_sq(v);

                if (distSqLine < range_sq) {
                    if (agentLeftOfLine < 0.0) {
                        agent.insert_obstacle_neighbor(node.get_obstacle(), range_sq);
                    }

                    // Try other side of line. 
                    this.query_obstacle_tree_recursive(agent, range_sq, (agentLeftOfLine >= 0.0 ? node.get_right() : node.get_left()));
                }
            }
        }
        else{
            return;
        }
    }

    query_visibility(start: Vector2, end: Vector2, radius: f32): bool{
        return this.query_visibility_recursive(start, end, radius, this.m_obstacle_tree);
    }

    query_visibility_recursive(q1: Vector2, q2: Vector2, radius: f32, node: ObstacleTreeNode | null): bool{
        if(node){
            let obstacle1 = node.get_obstacle();
            let obstacle2 = obstacle1.get_next_obstacle();
            if(obstacle2){
                let p1 = obstacle1.get_point();
                let p2 = obstacle2.get_point();
                const q1LeftOfI = left_of_points(p1, p2, q1);
                const q2LeftOfI = left_of_points(p1, p2, q2);
                let v = p2.subtract(p1);
                const invLengthI = 1.0 / abs_sq(v);

                if (q1LeftOfI >= 0.0 && q2LeftOfI >= 0.0) {
                    return this.query_visibility_recursive(q1, q2, radius, node.get_left()) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || this.query_visibility_recursive(q1, q2, radius, node.get_right()));
                }
                else if (q1LeftOfI <= 0.0 && q2LeftOfI <= 0.0) {
                    return this.query_visibility_recursive(q1, q2, radius, node.get_right()) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || this.query_visibility_recursive(q1, q2, radius, node.get_left()));
                }
                else if (q1LeftOfI >= 0.0 && q2LeftOfI <= 0.0) {
                    // One can see through obstacle from left to right.
                    return this.query_visibility_recursive(q1, q2, radius, node.get_left()) && this.query_visibility_recursive(q1, q2, radius, node.get_right());
                }
                else {
                    const point1LeftOfQ = left_of_points(q1, q2, p1);
                    const point2LeftOfQ = left_of_points(q1, q2, p2);
                    let v2 = q2.subtract(q1);
                    const invLengthQ = 1.0 / abs_sq(v2);

                    return (point1LeftOfQ * point2LeftOfQ >= 0.0 && sqr(point1LeftOfQ) * invLengthQ > sqr(radius) && sqr(point2LeftOfQ) * invLengthQ > sqr(radius) && this.query_visibility_recursive(q1, q2, radius, node.get_left()) && this.query_visibility_recursive(q1, q2, radius, node.get_right()));
                }
            }
            else{
                return false;
            }
        }
        else{
            return true;
        }
    }
}