from typing import Tuple
import math
from pathfinder.pyrvo.utilities import left_of

class Obstacle:
    def __init__(self):
        self._is_convex = False
        self._next_obstacle = None
        self._point = (0.0, 0.0)
        self._prev_obstacle = None
        self._unit_dir = (0.0, 0.0)

        self._id = 0

    def set_point(self, point: Tuple[float, float]):
        self._point = point

    def get_point(self) -> Tuple[float, float]:
        return self._point

    def set_prev_obstacle(self, obstacle):
        self._prev_obstacle = obstacle

    def get_prev_obstacle(self):
        return self._prev_obstacle

    def set_next_obstacle(self, obstacle):
        self._next_obstacle = obstacle

    def get_next_obstacle(self):
        return self._next_obstacle

    def set_unit_dir(self, point_01: Tuple[float, float], point_02: Tuple[float, float]):
        v = (point_01[0] - point_02[0], point_01[1] - point_02[1])
        l = math.sqrt(v[0]**2 + v[1]**2)
        self._unit_dir = (v[0] / l, v[1] / l)

    def set_unit_dir_value(self, unit_dir: Tuple[float, float]):
        self._unit_dir = unit_dir

    def get_unit_dir(self) -> Tuple[float, float]:
        return self._unit_dir

    def set_is_convex(self, point_01: Tuple[float, float], point_02: Tuple[float, float], point_03: Tuple[float, float]):
        self._is_convex = left_of(point_01, point_02, point_03) >= 0.0

    def set_is_convex_value(self, is_convex: bool):
        self._is_convex = is_convex

    def get_is_convex(self) -> bool:
        return self._is_convex

    def set_id(self, id: int):
        self._id = id

    def get_id(self) -> int:
        return self._id

    def __repr__(self) -> str:
        return "obstacle[" + str(self._id) + "]"
