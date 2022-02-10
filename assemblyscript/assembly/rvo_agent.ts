import { Vector2, Line, abs_sq, dot, det, normalize } from "./vector2";
import { List } from "./list";
import { Obstacle } from "./rvo_obstacle";
import { RVOSimulator } from "./rvo_simulator";
import { log_message, dist_sq_point_line_segment, sqr, RVO_EPSILON, RVO_INFINITY } from "./utilities";

function linearProgram1(lines: List<Line>, lineNo: i32, radius: f32, optVelocity: Vector2, directionOpt: bool, result: Vector2): bool{
    let line_no = lines[lineNo];
    let line_p = line_no.get_point();
    let line_dir = line_no.get_direction();
    const dotProduct: f32 = dot(line_p, line_dir);
    const discriminant = sqr(dotProduct) + sqr(radius) - abs_sq(line_p);
    if (discriminant < 0.0) {
        return false;
    }

    const sqrtDiscriminant = Mathf.sqrt(discriminant);
    var tLeft = -dotProduct - sqrtDiscriminant;
    var tRight = -dotProduct + sqrtDiscriminant;

    for (let i = 0; i < lineNo; ++i) {
        let line_i = lines[i];
        let line_i_p = line_i.get_point();
        let line_i_dir = line_i.get_direction();
        const denominator = det(line_dir, line_i_dir);
        const numerator = det(line_i_dir, line_p.subtract(line_i_p));

        if (Mathf.abs(denominator) <= RVO_EPSILON) {
            // Lines lineNo and i are (almost) parallel. 
            if (numerator < 0.0) {
                return false;
            }
            else {
                continue;
            }
        }

        const t = numerator / denominator;

        if (denominator >= 0.0) {
            // Line i bounds line lineNo on the right.
            tRight = Mathf.min(tRight, t);
        }
        else {
            // Line i bounds line lineNo on the left.
            tLeft = Mathf.max(tLeft, t);
        }

        if (tLeft > tRight) {
            return false;
        }
    }

    if (directionOpt) {
        // Optimize direction. 
        if (dot(optVelocity, line_dir) > 0.0) {
            // Take right extreme. 
            result.copy_from(line_p.add(line_dir.scale(tRight)));
        }
        else {
            // Take left extreme. 
            result.copy_from(line_p.add(line_dir.scale(tLeft)));
        }
    }
    else {
        // Optimize closest point. 
        const t = dot(line_dir, optVelocity.subtract(line_p));

        if (t < tLeft) {
            result.copy_from(line_p.add(line_dir.scale(tLeft)));
        }
        else if (t > tRight) {
            result.copy_from(line_p.add(line_dir.scale(tRight)));
        }
        else {
            result.copy_from(line_p.add(line_dir.scale(t)));
        }
    }

    return true;
}

function linearProgram2(lines: List<Line>, radius: f32, optVelocity: Vector2, directionOpt: bool, result: Vector2): i32{
    if (directionOpt) {
        // Optimize direction. Note that the optimization velocity is of unit
        // length in this case.
        result.copy_from(optVelocity.scale(radius));
    }
    else if (abs_sq(optVelocity) > sqr(radius)) {
        // Optimize closest point and outside circle.
        result.copy_from(normalize(optVelocity).scale(radius));
    }
    else {
        // Optimize closest point and inside circle.
        result.copy_from(optVelocity);
    }

    for (let i = 0, len = lines.length; i < len; ++i) {
        let line_i = lines[i];
        let line_i_p = line_i.get_point();
        let line_i_dir = line_i.get_direction();
        if (det(line_i_dir, line_i_p.subtract(result)) > 0.0) {
            // Result does not satisfy constraint i. Compute new optimal result.
            const tempResult = result;

            if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
                result.copy_from(tempResult);
                return i;
            }
        }
    }

    return lines.length;
}

function linearProgram3(lines: List<Line>, numObstLines: i32, beginLine: i32, radius: f32, result: Vector2): void{
    var distance = 0.0;

    const lines_count = lines.length;
    for (let i = beginLine; i < lines_count; ++i) {
        let line_i = lines[i];
        let line_i_p = line_i.get_point();
        let line_i_dir = line_i.get_direction();
        if (det(line_i_dir, line_i_p.subtract(result)) > distance) {
            // Result does not satisfy constraint of line i.
            let projLines = new List<Line>(numObstLines);
            for(let k = 0; k < numObstLines; k++){
                projLines.push(lines[k]);
            }

            for (let j = numObstLines; j < i; ++j) {
                let line = new Line();

                let line_j = lines[j];
                let line_j_dir = line_j.get_direction();
                let line_j_p = line_j.get_point();
                const determinant = det(line_i_dir, line_j_dir);

                if (Mathf.abs(determinant) <= RVO_EPSILON) {
                    // Line i and line j are parallel.
                    if (dot(line_i_dir, line_j_dir) > 0.0) {
                        // Line i and line j point in the same direction.
                        continue;
                    }
                    else {
                        // Line i and line j point in opposite direction.
                        line.set_point(line_i_p.add(line_j_p).scale(0.5));
                    }
                }
                else {
                    let ij_p = line_i_p.subtract(line_j_p);
                    let d = det(line_j_dir, ij_p) / determinant;

                    line.set_point(line_i_p.add(line_i_dir.scale(d)));
                }
                line.set_direction(normalize(line_j_dir.subtract(line_i_dir)));
                projLines.push(line);
            }

            let tempResult = result;

            if (linearProgram2(projLines, radius, new Vector2(-line_i_dir.y(), line_i_dir.x()), true, result) < projLines.length) {
                // This should in principle not happen.  The result is by definition
                // already in the feasible region of this linear program. If it fails,
                // it is due to small floating point error, and the current result is
                // kept.
                result.copy_from(tempResult);
            }
            
            distance = det(line_i_dir, line_i_p.subtract(result));
        }
    }
}

export class Agent{
    m_agent_neighbors_values: StaticArray<f32>;
    m_agent_neighbors_agents: StaticArray<Agent>;
    m_agent_neighbors_count: i32;
    m_max_neighbors: i32;
    m_max_speed: f32;
    m_neighbor_dist: f32;
    m_new_velocity: Vector2 = new Vector2();
    m_obstacle_neighbors_values: List<f32>;
    m_obstacle_neighbors_obstacles: List<Obstacle>;
    m_orca_lines: List<Line>;
    m_position: Vector2 = new Vector2();
    m_pref_velocity: Vector2 = new Vector2();
    m_radius: f32;
    m_sim: RVOSimulator;
    m_time_horizon: f32;
    m_time_horizon_obst: f32;
    m_velocity: Vector2 = new Vector2();
    m_id: i32;
    
    constructor(sim: RVOSimulator, 
                id: i32,
                position: Vector2, 
                velocity: Vector2,
                neighbor_dist: f32,
                max_neighbors: i32,
                time_horizon: f32,
                time_horizon_obst: f32,
                radius: f32,
                max_speed: f32) {
        this.m_sim = sim;
        this.m_max_neighbors = max_neighbors;
        this.m_max_speed = max_speed;
        this.m_neighbor_dist = neighbor_dist;
        this.m_time_horizon = time_horizon;
        this.m_time_horizon_obst = time_horizon_obst;
        this.m_radius = radius;

        this.m_agent_neighbors_values = new StaticArray<f32>(this.m_max_neighbors);
        this.m_agent_neighbors_agents = new StaticArray<Agent>(this.m_max_neighbors);
        this.m_agent_neighbors_count = 0;

        this.m_obstacle_neighbors_values = new List<f32>();
        this.m_obstacle_neighbors_obstacles = new List<Obstacle>();

        this.m_new_velocity = new Vector2();
        this.m_orca_lines = new List<Line>();
        this.m_position = position;
        this.m_pref_velocity = new Vector2();
        this.m_velocity = velocity;
        this.m_id = id;
    }

    @inline
    get_agent_neighbors(): StaticArray<Agent>{
        //copy to output array
        var to_return = new StaticArray<Agent>(this.m_agent_neighbors_count);
        for(let i = 0, len = this.m_agent_neighbors_count; i < len; i++){
            to_return[i] = this.m_agent_neighbors_agents[i];
        }
        return to_return;
    }

    @inline
    get_agent_agent_neighbor(index: i32): Agent{
        return this.m_agent_neighbors_agents[index];
    }

    @inline
    get_agent_neighbors_count(): i32{
        return this.m_agent_neighbors_count;
    }

    @inline
    get_obstacle_neighbor(index: i32): Obstacle{
        return this.m_obstacle_neighbors_obstacles[index];
    }

    get_num_obstacle_neighbors(): i32{
        return this.m_obstacle_neighbors_obstacles.length;
    }

    @inline
    get_id(): i32{
        return this.m_id;
    }

    @inline
    get_max_neighbors(): i32{
        return this.m_max_neighbors;
    }

    @inline
    set_max_neighbors(max_neighbors: i32): void{
        this.m_max_neighbors = max_neighbors;
    }

    @inline
    get_max_speed(): f32{
        return this.m_max_speed;
    }

    @inline
    set_max_speed(max_speed: f32): void{
        this.m_max_speed = max_speed;
    }

    @inline
    get_neighbor_distance(): f32{
        return this.m_neighbor_dist;
    }

    @inline
    set_neighbor_distance(neighbor_dist: f32): void{
        this.m_neighbor_dist = neighbor_dist;
    }

    @inline
    get_position(): Vector2{
        return this.m_position;
    }

    @inline
    set_position(position: Vector2): void{
        this.m_position = position;
    }

    @inline
    get_pref_velocity(): Vector2{
        return this.m_pref_velocity;
    }

    @inline
    set_pref_velocity(pref_velocity: Vector2): void{
        this.m_pref_velocity = pref_velocity;
    }

    @inline
    get_velocity(): Vector2{
        return this.m_velocity;
    }

    @inline
    set_velocity(velocity: Vector2): void{
        this.m_velocity = velocity;
    }

    @inline
    get_radius(): f32{
        return this.m_radius;
    }

    @inline
    set_radius(radius: f32): void{
        this.m_radius = radius;
    }

    @inline
    get_time_horizon(): f32{
        return this.m_time_horizon;
    }

    @inline
    set_time_horizon(time_horizon: f32): void{
        this.m_time_horizon = time_horizon;
    }

    @inline
    get_time_horizon_obst(): f32{
        return this.m_time_horizon_obst;
    }

    @inline
    set_time_horizon_obst(time_horizon_obst: f32): void{
        this.m_time_horizon_obst = time_horizon_obst;
    }

    compute_neighbors(): void{
        this.m_obstacle_neighbors_values.reset();
        this.m_obstacle_neighbors_obstacles.reset();
        const range_sq = sqr(this.m_time_horizon_obst * this.m_max_speed + this.m_radius);
        this.m_sim.m_kd_tree.compute_obstacle_neighbors(this, range_sq);

        //reset counter of neighbor agents
        this.m_agent_neighbors_count = 0;

        if (this.m_max_neighbors > 0) {
            const range_sq = sqr(this.m_neighbor_dist);
            this.m_sim.m_kd_tree.compute_agent_neighbors(this, range_sq);
        }
    }

    compute_new_velocity(delta_time: f32): void{
        this.m_orca_lines.reset();

        const invTimeHorizonObst: f32 = 1.0 / this.m_time_horizon_obst;

        // Create obstacle ORCA lines.
        for (let i = 0, len = this.m_obstacle_neighbors_values.length; i < len; ++i) {

            let obstacle1 = this.m_obstacle_neighbors_obstacles[i];
            let obstacle2 = obstacle1.get_next_obstacle();

            if(obstacle2){
                let relativePosition1 = obstacle1.get_point().subtract(this.m_position);
                let relativePosition2 = obstacle2.get_point().subtract(this.m_position);
                let alreadyCovered = false;

                for (let j = 0, jlen = this.m_orca_lines.length; j < jlen; ++j) {
                    let line_j = this.m_orca_lines[j];
                    let line_j_p = line_j.get_point();
                    let line_j_dir = line_j.get_direction();
                    if(det(relativePosition1.scale(invTimeHorizonObst).subtract(line_j_p), line_j_dir) - invTimeHorizonObst * this.m_radius >= -<f32>RVO_EPSILON  &&
                       det(relativePosition2.scale(invTimeHorizonObst).subtract(line_j_p), line_j_dir) - invTimeHorizonObst * this.m_radius >= -<f32>RVO_EPSILON){
                        alreadyCovered = true;
                        break;
                    }
                }

                if (alreadyCovered) {
                    continue;
                }

                const distSq1 = abs_sq(relativePosition1);
                const distSq2 = abs_sq(relativePosition2);

                const radiusSq = sqr(this.m_radius);

                const obstacleVector = obstacle2.get_point().subtract(obstacle1.get_point());
                const s: f32 = -1.0*dot(relativePosition1, obstacleVector) / abs_sq(obstacleVector);
                const distSqLine = abs_sq(relativePosition1.scale(-1.0).subtract(obstacleVector.scale(s)));

                let line: Line = new Line();

                if (s < 0.0 && distSq1 <= radiusSq) {
                    if (obstacle1.get_is_convex()) {
                        line.set_point_values(0.0, 0.0);
                        line.set_direction(normalize(new Vector2(-relativePosition1.y(), relativePosition1.x())));
                        this.m_orca_lines.push(line);
                    }
                    continue;
                }
                else if (s > 1.0 && distSq2 <= radiusSq) {
                    if (obstacle2.get_is_convex() && det(relativePosition2, obstacle2.get_unit_dir()) >= 0.0) {
                        line.set_point_values(0.0, 0.0);
                        line.set_direction(normalize(new Vector2(-relativePosition2.y(), relativePosition2.x())));
                        this.m_orca_lines.push(line);
                    }
                    continue;
                }
                else if (s >= 0.0 && s < 1.0 && distSqLine <= radiusSq) {
                    line.set_point_values(0.0, 0.0);
                    line.set_direction(obstacle1.get_unit_dir().scale(-1.0));
                    this.m_orca_lines.push(line);
                    continue;
                }

                let leftLegDirection = new Vector2();
                let rightLegDirection = new Vector2();

                if (s < 0.0 && distSqLine <= radiusSq) {
                    if (!obstacle1.get_is_convex()) {
                        continue;
                    }

                    obstacle2 = obstacle1;

                    const leg1 = Mathf.sqrt(distSq1 - radiusSq);
                    leftLegDirection = (new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * this.m_radius, relativePosition1.x() * this.m_radius + relativePosition1.y() * leg1)).scale(1.0 / distSq1);
                    rightLegDirection = (new Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * this.m_radius, -relativePosition1.x() * this.m_radius + relativePosition1.y() * leg1)).scale(1.0 / distSq1);
                }
                else if (s > 1.0 && distSqLine <= radiusSq) {
                    if (!obstacle2.get_is_convex()) {
                        continue;
                    }

                    obstacle1 = obstacle2;

                    const leg2 = Mathf.sqrt(distSq2 - radiusSq);
                    leftLegDirection = (new Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * this.m_radius, relativePosition2.x() * this.m_radius + relativePosition2.y() * leg2)).scale(1.0 / distSq2);
                    rightLegDirection = (new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * this.m_radius, -relativePosition2.x() * this.m_radius + relativePosition2.y() * leg2)).scale(1.0 / distSq2);
                }
                else {
                    if (obstacle1.get_is_convex()) {
                        const leg1 = Mathf.sqrt(distSq1 - radiusSq);
                        leftLegDirection = (new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * this.m_radius, relativePosition1.x() * this.m_radius + relativePosition1.y() * leg1)).scale(1.0 / distSq1);
                    }
                    else {
                        leftLegDirection = obstacle1.get_unit_dir().scale(-1.0);
                    }

                    if (obstacle2.get_is_convex()) {
                        const leg2 = Mathf.sqrt(distSq2 - radiusSq);
                        rightLegDirection = (new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * this.m_radius, -relativePosition2.x() * this.m_radius + relativePosition2.y() * leg2)).scale(1.0 / distSq2);
                    }
                    else {
                        rightLegDirection = obstacle1.get_unit_dir();
                    }
                }
                let leftNeighbor = obstacle1.get_prev_obstacle();
                if(leftNeighbor){
                    let isLeftLegForeign = false;
                    let isRightLegForeign = false;

                    let lud = leftNeighbor.get_unit_dir().scale(-1.0);
                    if (obstacle1.get_is_convex() && det(leftLegDirection, lud) >= 0.0) {
                        leftLegDirection = lud;
                        isLeftLegForeign = true;
                    }

                    let o2ud = obstacle2.get_unit_dir();
                    if (obstacle2.get_is_convex() && det(rightLegDirection, o2ud) <= 0.0) {
                        rightLegDirection = o2ud;
                        isRightLegForeign = true;
                    }

                    const leftCutoff = obstacle1.get_point().subtract(this.m_position).scale(invTimeHorizonObst);
                    const rightCutoff = obstacle2.get_point().subtract(this.m_position).scale(invTimeHorizonObst);
                    const cutoffVec = rightCutoff.subtract(leftCutoff);

                    const t: f32 = (obstacle1 == obstacle2 ? 0.5 : (dot(this.m_velocity.subtract(leftCutoff), cutoffVec) / abs_sq(cutoffVec)));
                    const tLeft = dot(this.m_velocity.subtract(leftCutoff), leftLegDirection);
                    const tRight = dot(this.m_velocity.subtract(rightCutoff), rightLegDirection);

                    if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0)) {
                        const unitW = normalize(this.m_velocity.subtract(leftCutoff));

                        line.set_direction_values(unitW.y(), -unitW.x());
                        line.set_point(leftCutoff.add(unitW.scale(this.m_radius * invTimeHorizonObst)));
                        this.m_orca_lines.push(line);
                        continue;
                    }
                    else if (t > 1.0 && tRight < 0.0) {
                        const unitW = normalize(this.m_velocity.subtract(rightCutoff));

                        line.set_direction_values(unitW.y(), -unitW.x());
                        line.set_point(rightCutoff.add(unitW.scale(this.m_radius * invTimeHorizonObst)));
                        this.m_orca_lines.push(line);
                        continue;
                    }

                    const distSqCutoff = ((t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? RVO_INFINITY : abs_sq(this.m_velocity.subtract(leftCutoff.add(cutoffVec.scale(t)))));
                    const distSqLeft = ((tLeft < 0.0) ? RVO_INFINITY : abs_sq(this.m_velocity.subtract(leftCutoff.add(leftLegDirection.scale(tLeft)))));
                    const distSqRight = ((tRight < 0.0) ? RVO_INFINITY : abs_sq(this.m_velocity.subtract(rightCutoff.add(rightLegDirection.scale(tRight)))));

                    if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
                        line.set_direction(obstacle1.get_unit_dir().scale(-1.0));
                        let line_dir = line.get_direction();
                        line.set_point(leftCutoff.add((new Vector2(-line_dir.y(), line_dir.x())).scale(this.m_radius * invTimeHorizonObst)));
                        this.m_orca_lines.push(line);
                        continue;
                    }
                    else if (distSqLeft <= distSqRight) {
                        if (isLeftLegForeign) {
                            continue;
                        }

                        line.set_direction(leftLegDirection);
                        line.set_point(leftCutoff.add((new Vector2(-leftLegDirection.y(), leftLegDirection.x())).scale(this.m_radius * invTimeHorizonObst)));
                        this.m_orca_lines.push(line);
                        continue;
                    }
                    else {
                        if (isRightLegForeign) {
                            continue;
                        }

                        line.set_direction(rightLegDirection.scale(-1.0));
                        let line_dir = line.get_direction();
                        line.set_point(rightCutoff.add((new Vector2(-line_dir.y(), line_dir.x())).scale(this.m_radius * invTimeHorizonObst)));
                        this.m_orca_lines.push(line);
                    }
                }
            }
        }
        const numObstLines = this.m_orca_lines.length;

        const invTimeHorizon: f32 = 1.0 / this.m_time_horizon;
        for (let i = 0, len = this.m_agent_neighbors_count; i < len; ++i) {
            let other = this.m_agent_neighbors_agents[i];

            const relativePosition = other.get_position().subtract(this.m_position);
            const relativeVelocity = this.m_velocity.subtract(other.get_velocity());
            const distSq = abs_sq(relativePosition);
            const combinedRadius = this.m_radius + other.get_radius();
            const combinedRadiusSq = sqr(combinedRadius);

            var line = new Line();
            let u = new Vector2();

            if (distSq > combinedRadiusSq) {
                const w = relativeVelocity.subtract(relativePosition.scale(invTimeHorizon));
                const wLengthSq = abs_sq(w);

                const dotProduct1 = dot(w, relativePosition);

                if (dotProduct1 < 0.0 && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
                    const wLength = Mathf.sqrt(wLengthSq);
                    const unitW = w.scale(1.0 / wLength);

                    line.set_direction_values(unitW.y(), -unitW.x());
                    u = unitW.scale(combinedRadius * invTimeHorizon - wLength);
                }
                else {
                    const leg = Mathf.sqrt(distSq - combinedRadiusSq);

                    if (det(relativePosition, w) > 0.0) {
                        line.set_direction((new Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg)).scale(1.0 / distSq));
                    }
                    else {
                        line.set_direction((new Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg)).scale(-1.0 / distSq));
                    }

                    const dotProduct2 = dot(relativeVelocity, line.get_direction());

                    u = line.get_direction().scale(dotProduct2).subtract(relativeVelocity);
                }
            }
            else {
                const invTimeStep: f32 = 1.0 / delta_time;

                const w = relativeVelocity.subtract(relativePosition.scale(invTimeStep));

                const wLength = w.length();
                const unitW = w.scale(1.0 / wLength);

                line.set_direction_values(unitW.y(), -unitW.x());
                u = unitW.scale(combinedRadius * invTimeStep - wLength);
            }

            line.set_point(this.m_velocity.add(u.scale(0.5)));
            this.m_orca_lines.push(line);
        }

        var lineFail = linearProgram2(this.m_orca_lines, this.m_max_speed, this.m_pref_velocity, false, this.m_new_velocity);
        
        if (lineFail < this.m_orca_lines.length) {
            linearProgram3(this.m_orca_lines, numObstLines, lineFail, this.m_max_speed, this.m_new_velocity);
        }
    }

    insert_agent_neighbor(agent: Agent, range_sq: f32): f32{
        if (this != agent) {
            let v = this.m_position.subtract(agent.get_position());
            const dist_sq = abs_sq(v);

            if (dist_sq < range_sq) {
                if (this.m_agent_neighbors_count < this.m_max_neighbors) {
                    this.m_agent_neighbors_values[this.m_agent_neighbors_count] = dist_sq;
                    this.m_agent_neighbors_agents[this.m_agent_neighbors_count] = agent;
                    this.m_agent_neighbors_count++;
                }

                let i = this.m_agent_neighbors_count - 1;

                while (i != 0 && dist_sq < this.m_agent_neighbors_values[i - 1]) {
                    this.m_agent_neighbors_values[i] = this.m_agent_neighbors_values[i - 1];
                    this.m_agent_neighbors_agents[i] = this.m_agent_neighbors_agents[i - 1];
                    --i;
                }

                this.m_agent_neighbors_values[i] = dist_sq;
                this.m_agent_neighbors_agents[i] = agent;

                if (this.m_agent_neighbors_count == this.m_max_neighbors) {
                    range_sq = this.m_agent_neighbors_values[this.m_agent_neighbors_count - 1];
                }
            }
        }

        return range_sq;
    }

    insert_obstacle_neighbor(obstacle: Obstacle, range_sq: f32): void{
        var next_obstacle = obstacle.get_next_obstacle();
        if(next_obstacle){
            const dist_sq: f32 = dist_sq_point_line_segment(obstacle.get_point(), next_obstacle.get_point(), this.m_position);
            if (dist_sq < range_sq) {
                this.m_obstacle_neighbors_values.push(dist_sq);
                this.m_obstacle_neighbors_obstacles.push(obstacle);
                var i: i32 = this.m_obstacle_neighbors_obstacles.length - 1;

                while (i != 0 && dist_sq < this.m_obstacle_neighbors_values[i - 1]) {
                    this.m_obstacle_neighbors_values[i] = this.m_obstacle_neighbors_values[i - 1];
                    this.m_obstacle_neighbors_obstacles[i] = this.m_obstacle_neighbors_obstacles[i - 1];
                    --i;
                }

                this.m_obstacle_neighbors_values[i] = dist_sq;
                this.m_obstacle_neighbors_obstacles[i] = obstacle;
            }
        }
    }

    update(delta_time: f32, move_agents: bool = true): void{
        this.m_velocity = this.m_new_velocity;
        if(move_agents){
            this.m_position.add_inplace_values(this.m_velocity.x() * delta_time, this.m_velocity.y() * delta_time);
        }
    }

    to_string(): string{
        return "agent[" + this.m_id.toString() + "]@" + this.m_position.to_string();
    }
}