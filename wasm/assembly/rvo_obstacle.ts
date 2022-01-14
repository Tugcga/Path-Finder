import { left_of } from "./utilities";
import { Vector2 } from "./vector2";

export class Obstacle{
    m_is_convex: bool = false;
    m_next_obstacle: Obstacle | null;
    m_point: Vector2 = new Vector2();
    m_prev_obstacle: Obstacle | null;
    m_unit_dir: Vector2 = new Vector2();
    m_id: i32 = -1;

    @inline
    set_point_values(x: f32, y: f32): void{
        this.m_point.set_values(x, y);
    }

    @inline
    set_point(point: Vector2): void{
        this.m_point = point;
    }

    @inline
    get_point_x(): f32{
        return this.m_point.x();
    }

    @inline
    get_point_y(): f32{
        return this.m_point.y();
    }

    @inline
    get_point(): Vector2{
        return this.m_point;
    }

    @inline
    set_prev_obstacle(obstacle: Obstacle | null): void{
        this.m_prev_obstacle = obstacle;
    }

    @inline
    get_prev_obstacle(): Obstacle | null{
        return this.m_prev_obstacle;
    }

    @inline
    set_next_obstacle(obstacle: Obstacle | null): void{
        this.m_next_obstacle = obstacle;
    }

    @inline
    get_next_obstacle(): Obstacle | null{
        return this.m_next_obstacle;
    }

    @inline
    set_unit_dir_point(a_x: f32, a_y: f32, 
                       b_x: f32, b_y: f32): void{
        const v_x = a_x - b_x;
        const v_y = a_y - b_y;
        const l = Mathf.sqrt(v_x*v_x + v_y*v_y);
        this.m_unit_dir.set_values(v_x / l, v_y / l);
    }

    @inline
    set_unit_dir_values(x: f32, y: f32): void{
        this.m_unit_dir.set_values(x, y);
    }

    @inline
    set_unit_dir(unit_dir: Vector2): void{
        this.m_unit_dir = unit_dir;
    }

    @inline
    get_unit_dir_x(): f32{
        return this.m_unit_dir.x();
    }

    @inline
    get_unit_dir_y(): f32{
        return this.m_unit_dir.y();
    }

    @inline
    get_unit_dir(): Vector2{
        return this.m_unit_dir;
    }

    @inline
    set_is_convex(a_x: f32, a_y: f32, 
                  b_x: f32, b_y: f32, 
                  c_x: f32, c_y: f32): void{
        this.m_is_convex = left_of(a_x, a_y, b_x, b_y, c_x, c_y) >= 0.0;
    }

    @inline
    set_is_convex_value(is_convex: bool): void{
        this.m_is_convex = is_convex;
    }

    @inline
    get_is_convex(): bool{
        return this.m_is_convex;
    }

    @inline
    set_id(id: i32): void{
        this.m_id = id;
    }

    @inline
    get_id(): i32{
        return this.m_id;
    }

    to_string(): string{
        return "obstacle[" + this.m_id.toString() + "]";
    }

    toString(): string{
        return this.to_string();
    }
}