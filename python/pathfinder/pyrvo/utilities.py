from typing import Tuple
import math

RVO_EPSILON = 0.00001
MAX_LEAF_SIZE = 10
INFINITY = float("inf")

def left_of(point_a: Tuple[float, float], point_b: Tuple[float, float], point_c: Tuple[float, float]) -> float:
    v1 = (point_a[0] - point_c[0], point_a[1] - point_c[1])
    v2 = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    return v1[0]*v2[1] - v1[1]*v2[0]

def abs_sq(vector: Tuple[float, float]) -> float:
    return vector[0]**2 + vector[1]**2

def dist_sq_point_line_segment(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    r: float = ((c[0]-a[0])*(b[0]-a[0]) + (c[1]-a[1])*(b[1]-a[1])) / abs_sq((b[0] - a[0], b[1] - a[1]))
    if r < 0.0:
        return abs_sq((c[0] - a[0], c[1] - a[1]))
    elif r > 1.0:
        return abs_sq((c[0] - b[0], c[1] - b[1]))
    else:
        return abs_sq((c[0] - a[0] - r*(b[0] - a[0]), c[1] - a[1] - r*(b[1] - a[1])))

def vector_difference(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])

def vector_sum(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])

def vector_scale(c: float, vector: Tuple[float, float]) -> Tuple[float, float]:
    return (vector[0] * c, vector[1] * c)

def vector_negate(vector: Tuple[float, float]) -> Tuple[float, float]:
    return (-1*vector[0], -1*vector[1])

def vector_length(vector: Tuple[float, float]) -> float:
    return math.sqrt(vector[0]**2 + vector[1]**2)

def dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]

def det(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0]*b[1] - a[1]*b[0]

def normalize(vector: Tuple[float, float]) -> Tuple[float, float]:
    l = math.sqrt(vector[0]**2 + vector[1]**2)
    return (vector[0] / l, vector[1] / l)

def line_to_point(line: Tuple[float, float, float, float]) -> Tuple[float, float]:
    return (line[0], line[1])

def line_to_direction(line: Tuple[float, float, float, float]) -> Tuple[float, float]:
    return (line[2], line[3])
