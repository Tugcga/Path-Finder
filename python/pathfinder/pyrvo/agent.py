from typing import List, Tuple, Optional
import math
from pathfinder.pyrvo.obstacle import Obstacle
from pathfinder.pyrvo.utilities import abs_sq, dist_sq_point_line_segment, vector_difference, vector_scale, vector_negate, vector_sum, vector_length, det, dot, normalize, line_to_point, line_to_direction, RVO_EPSILON, INFINITY


def linear_program1(lines: List[Tuple[float, float, float, float]],
                    lineNo: int,
                    radius: float,
                    optVelocity: Tuple[float, float],
                    directionOpt: bool,
                    result: Tuple[float, float],
                    is_log: bool=False) -> Tuple[bool, Tuple[float, float]]:
    line_no_point = ((lines[lineNo][0], lines[lineNo][1]))
    line_no_direction = (lines[lineNo][2], lines[lineNo][3])
    dotProduct: float = dot(line_no_point, line_no_direction)
    discriminant: float = dotProduct**2 + radius**2 - abs_sq(line_no_point)

    if (discriminant < 0.0):
        # Max speed circle fully invalidates line lineNo.
        return (False, result)

    sqrtDiscriminant: float = math.sqrt(discriminant)
    tLeft: float = -dotProduct - sqrtDiscriminant
    tRight: float = -dotProduct + sqrtDiscriminant

    for i in range(lineNo):
        line_i_point = (lines[i][0], lines[i][1])
        line_i_direction = (lines[i][2], lines[i][3])
        denominator: float = det(line_no_direction, line_i_direction)
        numerator: float = det(line_i_direction, vector_difference(line_no_point, line_i_point))

        if (abs(denominator) <= RVO_EPSILON):
            # Lines lineNo and i are (almost) parallel.
            if (numerator < 0.0):
                return (False, result)
            else:
                continue

        t: float = numerator / denominator

        if (denominator >= 0.0):
            # Line i bounds line lineNo on the right.
            tRight = min(tRight, t)
        else:
            # Line i bounds line lineNo on the left.
            tLeft = max(tLeft, t)

        if (tLeft > tRight):
            return (False, result)

    if (directionOpt):
        # Optimize direction.
        if (dot(optVelocity, line_no_direction) > 0.0):
            # Take right extreme.
            result = vector_sum(line_no_point, vector_scale(tRight, line_no_direction))
        else:
            # Take left extreme.
            result = vector_sum(line_no_point, vector_scale(tLeft, line_no_direction))
    else:
        # Optimize closest point.
        t = dot(line_to_direction(lines[lineNo]), vector_difference(optVelocity, line_no_point))

        if (t < tLeft):
            result = vector_sum(line_no_point, vector_scale(tLeft, line_no_direction))
        elif (t > tRight):
            result = vector_sum(line_no_point, vector_scale(tRight, line_no_direction))
        else:
            result = vector_sum(line_no_point, vector_scale(t, line_no_direction))
    return (True, result)

def linear_program2(lines: List[Tuple[float, float, float, float]],
                    radius: float,
                    opt_velocity: Tuple[float, float],
                    direction_opt: bool,
                    result: Tuple[float, float],
                    is_log: bool=False) -> Tuple[int, Tuple[float, float]]:
    if (direction_opt):
        # Optimize direction. Note that the optimization velocity is of unit
        # length in this case.
        result = vector_scale(radius, opt_velocity)
    elif (abs_sq(opt_velocity) > radius**2):
        # Optimize closest point and outside circle.
        result = vector_scale(radius, normalize(opt_velocity))
    else:
        # Optimize closest point and inside circle.
        result = opt_velocity

    for i in range(len(lines)):
        if (det((lines[i][2], lines[i][3]), vector_difference((lines[i][0], lines[i][1]), result)) > 0.0):
            # Result does not satisfy constraint i. Compute new optimal result.
            tempResult = result
            (a, b) = linear_program1(lines, i, radius, opt_velocity, direction_opt, result, is_log)
            result = b
            if (not a):
                result = tempResult
                return (i, result)

    return (len(lines), result)


def linear_program3(lines: List[Tuple[float, float, float, float]],
                    numObstLines: int,
                    beginLine: int,
                    radius: float,
                    result: Tuple[float, float],
                    is_log: bool=False) -> Tuple[float, float]:
    distance: float = 0.0

    for i in range(beginLine, len(lines)):
        line_i_point = line_to_point(lines[i])
        line_i_direction = line_to_direction(lines[i])
        if (det(line_i_direction, vector_difference(line_i_point, result)) > distance):
            # Result does not satisfy constraint of line i.
            projLines: List[Tuple[float, float, float, float]] = [(0.0, 0.0, 0.0, 0.0)] * numObstLines
            for k in range(numObstLines):
                projLines[k] = lines[k]

            for j in range(numObstLines, i):
                line_j_point = line_to_point(lines[j])
                line_j_direction = line_to_direction(lines[j])
                line = [0.0, 0.0, 0.0, 0.0]
                determinant: float = det(line_i_direction, line_j_direction)

                if (abs(determinant) <= RVO_EPSILON):
                    # Line i and line j are parallel.
                    if (dot(line_i_direction, line_j_direction) > 0.0):
                        # Line i and line j point in the same direction.
                        continue
                    else:
                        # Line i and line j point in opposite direction.
                        p = vector_scale(0.5, vector_sum(line_i_point, line_j_point))
                        line[0] = p[0]
                        line[1] = p[1]
                else:
                    dif = vector_difference(line_i_point, line_j_point)
                    d_et = det(line_j_direction, dif)
                    s2 = vector_scale(d_et / determinant, line_i_direction)
                    p = vector_sum(line_i_point, s2)
                    line[0] = p[0]
                    line[1] = p[1]

                d = normalize(vector_difference(line_j_direction, line_i_direction))
                line[2] = d[0]
                line[3] = d[1]
                projLines.append((line[0], line[1], line[2], line[3]))

            tempResult = result

            (a, b) = linear_program2(projLines, radius, (-line_i_direction[1], line_i_direction[0]), True, result, is_log)
            result = b
            if (a < len(projLines)):
                # This should in principle not happen.  The result is by definition
                # already in the feasible region of this linear program. If it fails,
                # it is due to small floating point error, and the current result is
                # kept.
                result = tempResult

            distance = det(line_i_direction, vector_difference(line_i_point, result))
    return result


class Agent:
    def __init__(self, simulator, id: int, default_agent, position: Tuple[float, float], velocity: Optional[Tuple[float, float]],
                 neighbor_dist: Optional[float] = None,
                 max_neighbors: Optional[int] = None,
                 time_horizon: Optional[float] = None,
                 time_horizon_obst: Optional[float] = None,
                 radius: Optional[float] = None,
                 max_speed: Optional[float] = None):
        self._sim = simulator

        self._max_neighbors = default_agent._max_neighbors if max_neighbors is None else max_neighbors
        self._max_speed = default_agent._max_speed if max_speed is None else max_speed
        self._neighbor_dist = default_agent._neighbor_dist if neighbor_dist is None else neighbor_dist
        self._time_horizon = default_agent._time_horizon if time_horizon is None else time_horizon
        self._time_horizon_obstacle = default_agent._time_horizon_obstacle if time_horizon_obst is None else time_horizon_obst
        self._radius = default_agent._radius if radius is None else radius

        self._agent_neighbors: List[Tuple[float, Agent]] = []
        self._obstacle_neighbors: List[Tuple[float, Obstacle]] = []
        self._new_velocity: Tuple[float, float] = (0.0, 0.0)
        self._orcaLines: List[Tuple[float, float, float, float]] = []  # each line is two vectors: point and direction

        self._position = position
        self._pref_velocity: Tuple[float, float] = (0.0, 0.0)
        self._velocity = velocity if velocity is not None else (0.0, 0.0)

        self._id = id

    def __repr__(self):
        return str(self._id)

    def set_parameters(self, max_neighbors: int,
                       max_speed: float,
                       neighbor_dist: float,
                       radius: float,
                       time_horizon: float,
                       time_horizon_obst: float,
                       velocity: Tuple[float, float]):
        self._max_neighbors = max_neighbors
        self._max_speed = max_speed
        self._neighbor_dist = neighbor_dist
        self._radius = radius
        self._time_horizon = time_horizon
        self._time_horizon_obstacle = time_horizon_obst
        self._velocity = velocity

    def get_agent_neighbors(self):
        return self._agent_neighbors

    def get_obstacle_neighbors(self):
        return self._obstacle_neighbors

    def get_id(self) -> int:
        return self._id

    def get_max_neighbors(self) -> int:
        return self._max_neighbors

    def set_max_neighbors(self, max_neighbors: int):
        self._max_neighbors = max_neighbors

    def get_max_speed(self) -> float:
        return self._max_speed

    def set_max_speed(self, max_speed: float):
        self._max_speed = max_speed

    def get_neighbor_distance(self) -> float:
        return self._neighbor_dist

    def set_neighbor_distance(self, neighbor_dist: float):
        self._neighbor_dist = neighbor_dist

    def get_position(self) -> Tuple[float, float]:
        return self._position

    def set_position(self, position: Tuple[float, float]):
        self._position = position

    def get_pref_velocity(self) -> Optional[Tuple[float, float]]:
        return self._pref_velocity

    def set_pref_velocity(self, pref_velocity: Tuple[float, float]):
        self._pref_velocity = pref_velocity

    def get_velocity(self) -> Tuple[float, float]:
        return self._velocity

    def set_velocity(self, velocity: Tuple[float, float]):
        self._velocity = velocity

    def get_radius(self) -> float:
        return self._radius

    def set_radius(self, radius: float):
        self._radius = radius

    def get_time_horizon(self) -> float:
        return self._time_horizon

    def set_time_horizon(self, time_horizon: float):
        self._time_horizon = time_horizon

    def get_time_horizon_obst(self) -> float:
        return self._time_horizon_obstacle

    def set_time_horizon_obst(self, time_horizon_obst: float):
        self._time_horizon_obstacle = time_horizon_obst

    def compute_neighbors(self):
        self._obstacle_neighbors = []
        range_sq: float = (self._time_horizon_obstacle * self._max_speed + self._radius)**2
        self._sim.compute_agent_obstacles_neighbors(self, range_sq)

        self._agent_neighbors = []
        if self._max_neighbors > 0:
            range_sq = self._neighbor_dist**2
            self._sim.compute_agent_agents_neighbors(self, range_sq)

    def compute_new_velocity(self, delta_time: float):
        # the main method in RVO
        self._orcaLines = []

        invTimeHorizonObst: float = 1.0 / self._time_horizon_obstacle

        # Create obstacle ORCA lines.
        for i in range(len(self._obstacle_neighbors)):
            obstacle1 = self._obstacle_neighbors[i][1]
            obstacle2 = obstacle1.get_next_obstacle()

            relativePosition1 = vector_difference(obstacle1.get_point(), self._position)
            relativePosition2 = vector_difference(obstacle2.get_point(), self._position)

            # Check if velocity obstacle of obstacle is already taken care of by
            # previously constructed obstacle ORCA lines.
            alreadyCovered: bool = False

            for j in range(len(self._orcaLines)):
                p = (self._orcaLines[j][0], self._orcaLines[j][1])
                d = (self._orcaLines[j][2], self._orcaLines[j][3])
                if det(vector_difference(vector_scale(invTimeHorizonObst, relativePosition1), p), d) - invTimeHorizonObst * self._radius >= -RVO_EPSILON and det(vector_difference(vector_scale(invTimeHorizonObst, relativePosition2), p), d) - invTimeHorizonObst * self._radius >= -RVO_EPSILON:
                    alreadyCovered = True
                    break

            if alreadyCovered:
                continue;

            # Not yet covered. Check for collisions.

            distSq1: float = abs_sq(relativePosition1)
            distSq2: float = abs_sq(relativePosition2)

            radiusSq: float = self._radius**2

            obstacleVector = vector_difference(obstacle2.get_point(), obstacle1.get_point())
            s: float = dot(vector_negate(relativePosition1), obstacleVector) / abs_sq(obstacleVector)
            distSqLine: float = abs_sq(vector_difference(vector_negate(relativePosition1), vector_scale(s, obstacleVector)))

            line = [0.0, 0.0, 0.0, 0.0]

            if (s < 0.0 and distSq1 <= radiusSq):
                # Collision with left vertex. Ignore if non-convex. 
                if (obstacle1.get_is_convex()):
                    line[0] = 0.0
                    line[1] = 0.0
                    n = normalize((-1 * relativePosition1[1], relativePosition1[0]))
                    line[2] = n[0]
                    line[3] = n[1]
                    self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue
            elif (s > 1.0 and distSq2 <= radiusSq):
                # Collision with right vertex. Ignore if non-convex
                # or if it will be taken care of by neighoring obstace 
                if (obstacle2.get_is_convex() and det(relativePosition2, obstacle2.get_unit_dir()) >= 0.0):
                    line[0] = 0.0
                    line[1] = 0.0
                    n = normalize((-1 * relativePosition2[1], relativePosition2[0]))
                    line[2] = n[0]
                    line[3] = n[1]
                    self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue
            elif (s >= 0.0 and s <= 1.0 and distSqLine <= radiusSq):
                # Collision with obstacle segment. 
                line[0] = 0.0
                line[1] = 0.0
                n = vector_negate(obstacle1.get_unit_dir())
                line[2] = n[0]
                line[3] = n[1]
                self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue

            # No collision.
            # Compute legs. When obliquely viewed, both legs can come from a single
            # vertex. Legs extend cut-off line when nonconvex vertex.
            leftLegDirection = (0.0, 0.0)
            rightLegDirection = (0.0, 0.0)

            if (s < 0.0 and distSqLine <= radiusSq):
                # Obstacle viewed obliquely so that left vertex
                # defines velocity obstacle.
                if (not obstacle1.get_is_convex()):
                    # Ignore obstacle.
                    continue

                obstacle2 = obstacle1

                leg1: float = math.sqrt(distSq1 - radiusSq)
                leftLegDirection = vector_scale(1.0 / distSq1, (relativePosition1[0] * leg1 - relativePosition1[1] * self._radius, relativePosition1[0] * self._radius + relativePosition1[1] * leg1))
                rightLegDirection = vector_scale(1.0 / distSq1, (relativePosition1[0] * leg1 + relativePosition1[1] * self._radius, -relativePosition1[0] * self._radius + relativePosition1[1] * leg1))
            elif (s > 1.0 and distSqLine <= radiusSq):
                # Obstacle viewed obliquely so that
                # right vertex defines velocity obstacle.
                if (not obstacle2.get_is_convex()):
                    # Ignore obstacle.
                    continue

                obstacle1 = obstacle2

                leg2:float = math.sqrt(distSq2 - radiusSq)
                leftLegDirection = vector_scale(1.0 / distSq2, (relativePosition2[0] * leg2 - relativePosition2[1] * self._radius, relativePosition2[0] * self._radius + relativePosition2[1] * leg2))
                rightLegDirection = vector_scale(1.0 / distSq2, (relativePosition2[0] * leg2 + relativePosition2[1] * self._radius, -relativePosition2[0] * self._radius + relativePosition2[1] * leg2))
            else:
                # Usual situation.
                if (obstacle1.get_is_convex()):
                    leg1 = math.sqrt(distSq1 - radiusSq)
                    leftLegDirection = vector_scale(1.0 / distSq1, (relativePosition1[0] * leg1 - relativePosition1[1] * self._radius, relativePosition1[0] * self._radius + relativePosition1[1] * leg1))
                else:
                    # Left vertex non-convex; left leg extends cut-off line.
                    leftLegDirection = vector_negate(obstacle1.get_unit_dir())

                if (obstacle2.get_is_convex()):
                    leg2 = math.sqrt(distSq2 - radiusSq)
                    rightLegDirection = vector_scale(1.0 / distSq2, (relativePosition2[0] * leg2 + relativePosition2[1] * self._radius, -relativePosition2[0] * self._radius + relativePosition2[1] * leg2))
                else:
                    # Right vertex non-convex; right leg extends cut-off line.
                    rightLegDirection = obstacle1.get_unit_dir()

            # Legs can never point into neighboring edge when convex vertex,
            # take cutoff-line of neighboring edge instead. If velocity projected on
            # "foreign" leg, no constraint is added.
            leftNeighbor = obstacle1.get_prev_obstacle()

            isLeftLegForeign = False
            isRightLegForeign = False

            if (obstacle1.get_is_convex() and det(leftLegDirection, vector_negate(leftNeighbor.get_unit_dir())) >= 0.0):
                # Left leg points into obstacle.
                leftLegDirection = vector_negate(leftNeighbor.get_unit_dir())
                isLeftLegForeign = True

            if (obstacle2.get_is_convex() and det(rightLegDirection, obstacle2.get_unit_dir()) <= 0.0):
                # Right leg points into obstacle. 
                rightLegDirection = obstacle2.get_unit_dir()
                isRightLegForeign = True

            # Compute cut-off centers.
            leftCutoff = vector_scale(invTimeHorizonObst, vector_difference(obstacle1.get_point(), self._position))
            rightCutoff = vector_scale(invTimeHorizonObst, vector_difference(obstacle2.get_point(), self._position))
            cutoffVec = vector_difference(rightCutoff, leftCutoff)

            # Project current velocity on velocity obstacle.

            # Check if current velocity is projected on cutoff circles.
            t: float = 0.5 if obstacle1 == obstacle2 else dot(vector_difference(self._velocity, leftCutoff), cutoffVec) / abs_sq(cutoffVec)
            tLeft: float = dot(vector_difference(self._velocity, leftCutoff), leftLegDirection)
            tRight: float = dot(vector_difference(self._velocity, rightCutoff), rightLegDirection)

            if ((t < 0.0 and tLeft < 0.0) or (obstacle1 == obstacle2 and tLeft < 0.0 and tRight < 0.0)):
                # Project on left cut-off circle. 
                unitW = normalize(vector_difference(self._velocity, leftCutoff))
                line[2] = unitW[1]
                line[3] = -unitW[0]
                p = vector_sum(leftCutoff, vector_scale(self._radius * invTimeHorizonObst, unitW))
                line[0] = p[0]
                line[1] = p[1]
                self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue
            elif (t > 1.0 and tRight < 0.0):
                # Project on right cut-off circle.
                unitW = normalize(vector_difference(self._velocity, rightCutoff))
                line[2] = unitW[1]
                line[3] = -unitW[0]
                p = vector_sum(rightCutoff, vector_scale(self._radius * invTimeHorizonObst, unitW))
                line[0] = p[0]
                line[1] = p[1]
                self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue

            # Project on left leg, right leg, or cut-off line, whichever is closest
            # to velocity.
            distSqCutoff: float = INFINITY if (t < 0.0 or t > 1.0 or obstacle1 == obstacle2) else abs_sq(vector_difference(self._velocity, vector_sum(leftCutoff, vector_scale(t, cutoffVec))))
            distSqLeft: float = INFINITY if (tLeft < 0.0) else abs_sq(vector_difference(self._velocity, (vector_sum(leftCutoff, vector_scale(tLeft, leftLegDirection)))))
            distSqRight: float = INFINITY if (tRight < 0.0) else abs_sq(vector_difference(self._velocity, vector_sum(rightCutoff, vector_scale(tRight, rightLegDirection))))

            if (distSqCutoff <= distSqLeft and distSqCutoff <= distSqRight):
                # Project on cut-off line. 
                d = vector_negate(obstacle1.get_unit_dir())
                p = vector_sum(leftCutoff, vector_scale(self._radius * invTimeHorizonObst, (-d[1], d[0])))
                line[0] = p[0]
                line[1] = p[1]
                line[2] = d[0]
                line[3] = d[1]
                self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue
            elif (distSqLeft <= distSqRight):
                # Project on left leg.
                if (isLeftLegForeign):
                    continue

                line[2] = leftLegDirection[0]
                line[3] = leftLegDirection[1]
                p = vector_sum(leftCutoff, vector_scale(self._radius * invTimeHorizonObst, (-leftLegDirection[1], leftLegDirection[0])))
                line[0] = p[0]
                line[1] = p[1]
                self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue
            else:
                # Project on right leg.
                if (isRightLegForeign):
                    continue
                line[2] = -rightLegDirection[0]
                line[3] = -rightLegDirection[1]
                p = vector_sum(rightCutoff, vector_scale(self._radius * invTimeHorizonObst, (rightLegDirection[1], -rightLegDirection[0])))
                line[0] = p[0]
                line[1] = p[1]
                self._orcaLines.append((line[0], line[1], line[2], line[3]))
                continue

        numObstLines = len(self._orcaLines)
        invTimeHorizon: float = 1.0 / self._time_horizon

        # Create agent ORCA lines.
        for i in range(len(self._agent_neighbors)):
            other: Agent = self._agent_neighbors[i][1]

            relativePosition = vector_difference(other.get_position(), self._position)
            relativeVelocity = vector_difference(self._velocity, other.get_velocity())
            distSq: float = abs_sq(relativePosition)
            combinedRadius: float = self._radius + other.get_radius()
            combinedRadiusSq: float = combinedRadius**2

            line = [0.0, 0.0, 0.0, 0.0]
            u = (0.0, 0.0)

            if (distSq > combinedRadiusSq):
                # No collision.
                w = vector_difference(relativeVelocity, vector_scale(invTimeHorizon, relativePosition))
                # Vector from cutoff center to relative velocity.
                wLengthSq: float = abs_sq(w)
                dotProduct1: float = dot(w, relativePosition)

                if (dotProduct1 < 0.0 and (dotProduct1**2) > combinedRadiusSq * wLengthSq):
                    # Project on cut-off circle.
                    wLength: float = math.sqrt(wLengthSq)
                    unitW = vector_scale(1.0/wLength, w)

                    line[2] = unitW[1]
                    line[3] = -unitW[0]
                    u = vector_scale(combinedRadius * invTimeHorizon - wLength, unitW)
                else:
                    # Project on legs.
                    leg: float = math.sqrt(distSq - combinedRadiusSq)

                    if (det(relativePosition, w) > 0.0):
                        # Project on left leg.
                        d = (relativePosition[0] * leg - relativePosition[1] * combinedRadius, relativePosition[0] * combinedRadius + relativePosition[1] * leg)
                        line[2] = d[0] / distSq
                        line[3] = d[1] / distSq
                    else:
                        # Project on right leg.
                        d = (relativePosition[0] * leg + relativePosition[1] * combinedRadius, -relativePosition[0] * combinedRadius + relativePosition[1] * leg)
                        line[2] = -d[0] / distSq
                        line[3] = -d[1] / distSq

                    dotProduct2: float = dot(relativeVelocity, (line[2], line[3]))
                    u = vector_difference((dotProduct2 * line[2], dotProduct2 * line[3]), relativeVelocity)
            else:
                # Collision. Project on cut-off circle of time timeStep.
                invTimeStep: float = 1.0 / delta_time

                # Vector from cutoff center to relative velocity.
                w = vector_difference(relativeVelocity, vector_scale(invTimeStep, relativePosition))

                wLength = vector_length(w)
                if wLength > 0.0001:
                    unitW = vector_scale(1.0 / wLength, w)
                else:
                    unitW = (0.0, 0.0)

                line[2] = unitW[1]
                line[3] = -unitW[0]
                u = vector_scale((combinedRadius * invTimeStep - wLength), unitW)

            if self._velocity is not None:
                line[0] = self._velocity[0] + 0.5 * u[0]
                line[1] = self._velocity[1] + 0.5 * u[1]
            self._orcaLines.append((line[0], line[1], line[2], line[3]))

        (lineFail, b) = linear_program2(self._orcaLines, self._max_speed, self._pref_velocity, False, self._new_velocity)
        self._new_velocity = b

        if (lineFail < len(self._orcaLines)):
            self._new_velocity = linear_program3(self._orcaLines, numObstLines, lineFail, self._max_speed, self._new_velocity)

    def insert_agent_neighbor(self, agent, range_square: float) -> float:
        if agent != self:
            agent_pos = agent.get_position()
            dist_sq = abs_sq((self._position[0] - agent_pos[0], self._position[1] - agent_pos[1]))
            if dist_sq < range_square:
                if len(self._agent_neighbors) < self._max_neighbors:
                    self._agent_neighbors.append((dist_sq, agent))

                i: int = len(self._agent_neighbors) - 1
                while i != 0 and dist_sq < self._agent_neighbors[i - 1][0]:
                    self._agent_neighbors[i] = self._agent_neighbors[i - 1]
                    i -= 1

                self._agent_neighbors[i] = (dist_sq, agent)
                if len(self._agent_neighbors) == self._max_neighbors:
                    range_square = self._agent_neighbors[-1][0]
        return range_square

    def insert_obstacle_neighbor(self, obstacle, range_square: float):
        next_obstacle = obstacle.get_next_obstacle()
        dist_sq: float = dist_sq_point_line_segment(obstacle.get_point(), next_obstacle.get_point(), self._position)
        if dist_sq < range_square:
            self._obstacle_neighbors.append((dist_sq, obstacle))
            i = len(self._obstacle_neighbors) - 1
            while i != 0 and dist_sq < self._obstacle_neighbors[i - 1][0]:
                self._obstacle_neighbors[i] = self._obstacle_neighbors[i - 1]
                i -= 1
            self._obstacle_neighbors[i] = (dist_sq, obstacle)

    def update(self, delta_time: float, move_agents: bool = True):
        self._velocity = self._new_velocity
        if move_agents:
            self._position = (self._position[0] + self._velocity[0] * delta_time, self._position[1] + self._velocity[1] * delta_time)