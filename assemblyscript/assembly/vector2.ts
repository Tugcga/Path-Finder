
export class Line{
    m_point: Vector2;
    m_direction: Vector2;

    constructor(point: Vector2 = new Vector2(), direction: Vector2 = new Vector2()) {
        this.m_point = point;
        this.m_direction = direction;
    }

    @inline
    set_point(point: Vector2): void{
        this.m_point = point;
    }

    @inline
    set_point_values(x: f32, y: f32): void{
        this.m_point.set_values(x, y);
    }

    @inline
    set_direction(direction: Vector2): void{
        this.m_direction = direction;
    }

    @inline
    set_direction_values(x: f32, y: f32): void{
        this.m_direction.set_values(x, y);
    }

    @inline
    get_point(): Vector2{
        return this.m_point;
    }

    @inline
    get_direction(): Vector2{
        return this.m_direction;
    }
}

export class Vector2{
    m_x: f32;
    m_y: f32;

    constructor(x: f32 = 0.0, y: f32 = 0.0) {
        this.m_x = x;
        this.m_y = y;
    }

    @inline
    x(): f32{
        return this.m_x;
    }

    @inline
    y(): f32{
        return this.m_y;
    }

    @inline
    set_values(x: f32, y: f32): void{
        this.m_x = x;
        this.m_y = y;
    }

    @inline
    length(): f32{
        return Mathf.sqrt(this.m_x * this.m_x + this.m_y * this.m_y);
    }

    @inline
    add(other: Vector2): Vector2{
        return new Vector2(this.m_x + other.x(), this.m_y + other.y());
    }

    @inline
    add_inplace(other: Vector2): void{
        this.m_x += other.x();
        this.m_y += other.y();
    }

    @inline
    add_inplace_values(x: f32, y: f32): void{
        this.m_x += x;
        this.m_y += y;
    }

    @inline
    subtract(other: Vector2): Vector2{
        return new Vector2(this.m_x - other.x(), this.m_y - other.y());
    }

    @inline
    subtract_inplace(other: Vector2): void{
        this.m_x -= other.x();
        this.m_y -= other.y();
    }

    @inline
    scale(value: f32): Vector2{
        return new Vector2(value * this.m_x, value * this.m_y);
    }

    @inline
    copy_from(other: Vector2): void{
        this.m_x = other.x();
        this.m_y = other.y();
    }

    to_string(): string{
        return "(" + this.m_x.toString() + ", " + this.m_y.toString() + ")";
    }

    toString(): string{
        return this.to_string();
    }
}

@inline
export function abs_sq(a: Vector2): f32{
    return a.x()*a.x() + a.y()*a.y();
}

@inline
export function dot(a: Vector2, b: Vector2): f32{
    return a.x() * b.x() + a.y() * b.y();
}

@inline
export function det(a: Vector2, b: Vector2): f32{
    return a.x() * b.y() - a.y() * b.x();
}

@inline
export function normalize(vector: Vector2): Vector2{
    const x = vector.x();
    const y = vector.y();
    const l = Mathf.sqrt(x*x + y*y);
    return new Vector2(x / l, y / l);
}