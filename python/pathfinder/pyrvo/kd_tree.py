from typing import List, Tuple, Optional
from pathfinder.pyrvo.obstacle import Obstacle
from pathfinder.pyrvo.agent import Agent
from pathfinder.pyrvo.utilities import left_of, abs_sq, RVO_EPSILON, MAX_LEAF_SIZE

class AgentTreeNode:
    def __init__(self):
        self._begin = None
        self._end = None
        self._left = None
        self._right = None

        self._max_x = None
        self._max_y = None
        self._min_x = None
        self._min_y = None

    def __repr__(self):
        return "<[" + str(self._begin) + ", " + str(self._end) + ", " + str(self._left) + ", " + str(self._right) + "], [" + str(self._max_x) + ", " + str(self._max_y) + ", " + str(self._min_x) + ", " + str(self._min_y) + "]>"


class ObstacleTreeNode:
    def __init__(self):
        self._left = None
        self._right = None
        self._obstacle = None

    def set_obstacle(self, obstacle):
        self._obstacle = obstacle

    def get_obstacle(self) -> Obstacle:
        return self._obstacle

    def set_left(self, node):
        self._left = node

    def get_left(self):
        return self._left

    def set_right(self, node):
        self._right = node

    def get_right(self):
        return self._right

    def to_string(self):
        return "<" + str(self._obstacle.get_id()) + ">:[" + (self._left.to_string() if self._left is not None else "None") + "-" + (self._right.to_string() if self._right is not None else "None") + "]"


class KDTree:
    def __init__(self, simulator):
        self._sim = simulator;
        self._agents = []
        self._agent_tree = []
        self._obstacle_tree = None

    def build_agent_tree(self, force_update: bool):
        if force_update:
            self._agents = [a for a in self._sim.get_all_agents()]
            self._agent_tree = [None]*(2*len(self._agents) - 1 if len(self._agents) > 0 else 0)
        # start building
        if len(self._agents) > 0:
            self._build_agent_tree_recirsive(0, len(self._agents), 0)

    def delete_agent(self, agent_index: int):
        pass

    def _build_agent_tree_recirsive(self, begin: int, end: int, node: int):
        self._agent_tree[node] = AgentTreeNode()
        self._agent_tree[node]._begin = begin
        self._agent_tree[node]._end = end
        self._agent_tree[node]._min_x = self._agents[begin].get_position()[0]
        self._agent_tree[node]._max_x = self._agents[begin].get_position()[0]
        self._agent_tree[node]._min_y = self._agents[begin].get_position()[1]
        self._agent_tree[node]._max_y = self._agents[begin].get_position()[1]

        for i in range(begin + 1, end):
            self._agent_tree[node]._max_x = max(self._agent_tree[node]._max_x, self._agents[i].get_position()[0])
            self._agent_tree[node]._min_x = min(self._agent_tree[node]._min_x, self._agents[i].get_position()[0])

            self._agent_tree[node]._max_y = max(self._agent_tree[node]._max_y, self._agents[i].get_position()[1])
            self._agent_tree[node]._min_y = min(self._agent_tree[node]._min_y, self._agents[i].get_position()[1])

        if (end - begin) > MAX_LEAF_SIZE:
            is_vertical = self._agent_tree[node]._max_x - self._agent_tree[node]._min_x > self._agent_tree[node]._max_y - self._agent_tree[node]._min_y
            split_value = 0.5 * (self._agent_tree[node]._max_x + self._agent_tree[node]._min_x) if is_vertical else 0.5 * (self._agent_tree[node]._max_y + self._agent_tree[node]._min_y)
            left = begin
            right = end
            while left < right:
                while left < right and (self._agents[left].get_position()[0] if is_vertical else self._agents[left].get_position()[1]) < split_value:
                    left += 1
                while right > left and (self._agents[right - 1].get_position()[0] if is_vertical else self._agents[right - 1].get_position()[1]) >= split_value:
                    right -= 1

                if left < right:
                    self._agents[left], self._agents[right-1] = self._agents[right-1], self._agents[left]
                    left += 1
                    right -= 1
            if left == begin:
                left += 1
                right += 1

            self._agent_tree[node]._left = node + 1
            self._agent_tree[node]._right = node + 2 * (left - begin)

            self._build_agent_tree_recirsive(begin, left, self._agent_tree[node]._left)
            self._build_agent_tree_recirsive(left, end, self._agent_tree[node]._right)

    def build_obstacle_tree(self):
        self._delete_obstacle_tree(self._obstacle_tree)
        obstacles_count = self._sim.get_obstacles_count()
        obstacles = [None] * obstacles_count
        for i in range(obstacles_count):
            obstacles[i] = self._sim.get_obstacle(i)

        self._obstacle_tree = self._build_obstacle_tree_recursive(obstacles)


    def _build_obstacle_tree_recursive(self, obstacles: List[Optional[Obstacle]]):
        if len(obstacles) == 0:
            return None
        else:
            node = ObstacleTreeNode()
            optimal_split: int = 0
            min_left: int = len(obstacles)
            min_right: int = len(obstacles)

            for i in range(len(obstacles)):
                left_size: int = 0
                right_size: int = 0
                obstacle_i1 = obstacles[i]
                if obstacle_i1 is not None:
                    obstacle_i2 = obstacle_i1.get_next_obstacle()

                    # Compute optimal split node
                    for j in range(len(obstacles)):
                        if i == j:
                            continue
                        obstacle_j1 = obstacles[j]
                        if obstacle_j1 is not None:
                            obstacle_j2 = obstacle_j1.get_next_obstacle()
                            j1_left_of_i: float = left_of(obstacle_i1.get_point(), obstacle_i2.get_point(), obstacle_j1.get_point());
                            j2_left_of_i: float = left_of(obstacle_i1.get_point(), obstacle_i2.get_point(), obstacle_j2.get_point());
                            if j1_left_of_i >= -RVO_EPSILON and j2_left_of_i >= -RVO_EPSILON:
                                left_size += 1
                            elif j1_left_of_i <= RVO_EPSILON and j2_left_of_i <= RVO_EPSILON:
                                right_size += 1
                            else:
                                left_size +=1
                                right_size += 1

                            if (max(left_size, right_size), min(left_size, right_size)) >= (max(min_left, min_right), min(min_left, min_right)):
                                break
                    if (max(left_size, right_size), min(left_size, right_size)) < (max(min_left, min_right), min(min_left, min_right)):
                        min_left = left_size
                        min_right = right_size
                        optimal_split = i

            # Build split node
            left_obstacles: List[Optional[Obstacle]] = [None]*min_left
            right_obstacles: List[Optional[Obstacle]] = [None]*min_right

            left_counter: int = 0
            right_counter: int = 0

            obstacle_i1 = obstacles[optimal_split]
            if obstacle_i1 is not None:
                obstacle_i2 = obstacle_i1.get_next_obstacle()
                for j in range(len(obstacles)):
                    if j == optimal_split:
                        continue

                    obstacle_j1 = obstacles[j]
                    if obstacle_j1 is not None:
                        obstacle_j2 = obstacle_j1.get_next_obstacle()

                        j1_left_of_i = left_of(obstacle_i1.get_point(), obstacle_i2.get_point(), obstacle_j1.get_point());
                        j2_left_of_i = left_of(obstacle_i1.get_point(), obstacle_i2.get_point(), obstacle_j2.get_point());

                        if j1_left_of_i >= -RVO_EPSILON and j2_left_of_i >= -RVO_EPSILON:
                            left_obstacles[left_counter] = obstacles[j]
                            left_counter += 1
                        elif j1_left_of_i <= RVO_EPSILON and j2_left_of_i <= RVO_EPSILON:
                            right_obstacles[right_counter] = obstacles[j]
                            right_counter += 1
                        else:
                            # Split obstacle j
                            pi1 = obstacle_i1.get_point()
                            pi2 = obstacle_i2.get_point()
                            pj1 = obstacle_j1.get_point()
                            pj2 = obstacle_j2.get_point()
                            v1 = (pi2[0] - pi1[0], pi2[1] - pi1[1])
                            v12 = (pj1[0] - pi1[0], pj1[1] - pi1[1])
                            v22 = (pj1[0] - pj2[0], pj1[1] - pj2[1])
                            t: float = (v1[0]*v12[1] - v1[1]*v12[0]) / (v1[0]*v22[1] - v1[1]*v22[0])
                            split_point = (pj1[0] + t * (pj2[0] - pj1[0]), pj1[1] + t * (pj2[1] - pj1[1]))

                            new_obstacle = Obstacle()
                            new_obstacle.set_point(split_point)
                            new_obstacle.set_prev_obstacle(obstacle_j1)
                            new_obstacle.set_next_obstacle(obstacle_j2)
                            new_obstacle.set_is_convex_value(True)
                            new_obstacle.set_unit_dir_value(obstacle_j1.get_unit_dir())
                            new_obstacle.set_id(self._sim.get_obstacles_count())

                            self._sim.add_obstacle_object(new_obstacle)

                            obstacle_j1.set_next_obstacle(new_obstacle)
                            obstacle_j2.set_prev_obstacle(new_obstacle)

                            if j1_left_of_i > 0.0:
                                left_obstacles[left_counter] = obstacle_j1
                                left_counter += 1
                                right_obstacles[right_counter] = new_obstacle
                                right_counter += 1
                            else:
                                right_obstacles[right_counter] = obstacle_j1
                                right_counter += 1
                                left_obstacles[left_counter] = new_obstacle
                                left_counter += 1

            node.set_obstacle(obstacle_i1)
            node.set_left(self._build_obstacle_tree_recursive(left_obstacles))
            node.set_right(self._build_obstacle_tree_recursive(right_obstacles))
            return node


    def compute_agent_neighbors(self, agent: Agent, range_square: float):
        range_square = self._query_agent_tree_recursive(agent, range_square, 0)

    def compute_obstacle_neighbors(self, agent: Agent, range_square: float):
        self._query_obstacle_tree_recursive(agent, range_square, self._obstacle_tree)

    def _delete_obstacle_tree(self, node: ObstacleTreeNode):
        pass

    def _query_agent_tree_recursive(self, agent: Agent, range_square: float, node: int) -> float:
        if self._agent_tree[node]._end - self._agent_tree[node]._begin <= MAX_LEAF_SIZE:
            for i in range(self._agent_tree[node]._begin, self._agent_tree[node]._end):
                range_square = agent.insert_agent_neighbor(self._agents[i], range_square)
        else:
            dist_sq_left: float = (max(0.0, self._agent_tree[self._agent_tree[node]._left]._min_x - agent.get_position()[0]))**2 + (max(0.0, agent.get_position()[0] - self._agent_tree[self._agent_tree[node]._left]._max_x))**2 + (max(0.0, self._agent_tree[self._agent_tree[node]._left]._min_y - agent.get_position()[1]))**2 + (max(0.0, agent.get_position()[1] - self._agent_tree[self._agent_tree[node]._left]._max_y))**2
            dist_sq_right: float = (max(0.0, self._agent_tree[self._agent_tree[node]._right]._min_x - agent.get_position()[0]))**2 + (max(0.0, agent.get_position()[0] - self._agent_tree[self._agent_tree[node]._right]._max_x))**2 + (max(0.0, self._agent_tree[self._agent_tree[node]._right]._min_y - agent.get_position()[1]))**2 + (max(0.0, agent.get_position()[1] - self._agent_tree[self._agent_tree[node]._right]._max_y))**2
            if dist_sq_right > dist_sq_left:
                if dist_sq_left < range_square:
                    range_square = self._query_agent_tree_recursive(agent, range_square, self._agent_tree[node]._left)
                    if dist_sq_right < range_square:
                        range_square = self._query_agent_tree_recursive(agent, range_square, self._agent_tree[node]._right)
            else:
                if dist_sq_right < range_square:
                    range_square = self._query_agent_tree_recursive(agent, range_square, self._agent_tree[node]._right)
                    if dist_sq_left < range_square:
                        range_square = self._query_agent_tree_recursive(agent, range_square, self._agent_tree[node]._left)
        return range_square

    def _query_obstacle_tree_recursive(self, agent: Agent, range_square: float, node: ObstacleTreeNode):
        if node is None:
            return None
        else:
            obstacle_01 = node.get_obstacle()
            obstacle_02 = obstacle_01.get_next_obstacle()
            agent_left_of_line: float = left_of(obstacle_01.get_point(), obstacle_02.get_point(), agent.get_position())

            self._query_obstacle_tree_recursive(agent, range_square, node.get_left() if agent_left_of_line >= 0.0 else node.get_right())

            p1 = obstacle_01.get_point()
            p2 = obstacle_02.get_point()
            dist_sq_line: float = (agent_left_of_line**2) / abs_sq((p2[0] - p1[0], p2[1] - p1[1]))
            if dist_sq_line < range_square:
                if agent_left_of_line < 0.0:
                    agent.insert_obstacle_neighbor(node.get_obstacle(), range_square)
                self._query_obstacle_tree_recursive(agent, range_square, node.get_right() if agent_left_of_line >= 0.0 else node.get_left())


    def query_visibility(self, start: Tuple[float, float], end: Tuple[float, float], radius: float) -> bool:
        return self._query_visibility_recursive(start, end, radius, self._obstacle_tree)

    def _query_visibility_recursive(self, q1: Tuple[float, float], q2: Tuple[float, float], radius: float, node: ObstacleTreeNode) -> bool:
        if node is None:
            return True
        else:
            obstacle_01 = node.get_obstacle()
            obstacle_02 = obstacle_01.get_next_obstacle()

            q1_left_of_i = left_of(obstacle_01.get_point(), obstacle_02.get_point(), q1)
            q2_left_of_i = left_of(obstacle_01.get_point(), obstacle_02.get_point(), q2)
            p1 = obstacle_01.get_point()
            p2 = obstacle_02.get_point()
            inv_length_i = 1.0 / abs_sq((p2[0] - p1[0], p2[1] - p1[1]))

            if q1_left_of_i >= 0.0 and q2_left_of_i >= 0.0:
                return self._query_visibility_recursive(q1, q2, radius, node.get_left()) and (((q1_left_of_i**2) * inv_length_i >= radius**2 and (q2_left_of_i**2) * inv_length_i >= radius**2) or self._query_visibility_recursive(q1, q2, radius, node.get_right()))
            elif q1_left_of_i <= 0.0 and q2_left_of_i <= 0.0:
                return self._query_visibility_recursive(q1, q2, radius, node.get_right()) and (((q1_left_of_i**2) * inv_length_i >= radius**2 and (q2_left_of_i**2) * inv_length_i >= radius**2) or self._query_visibility_recursive(q1, q2, radius, node.get_left()))
            elif q1_left_of_i >= 0.0 and q2_left_of_i <= 0.0:
                return self._query_visibility_recursive(q1, q2, radius, node.get_left()) and self._query_visibility_recursive(q1, q2, radius, node.get_right())
            else:
                point_01_left_of_q: float = left_of(q1, q2, obstacle_01.get_point())
                point_02_left_of_q: float = left_of(q1, q2, obstacle_02.get_point())
                inv_length_q: float = 1.0 / abs_sq((q2[0] - q1[0], q2[1] - q1[1]))

                return (point_01_left_of_q * point_02_left_of_q >= 0.0 and (point_01_left_of_q**2) * inv_length_q > radius**2 and (point_02_left_of_q**2) * inv_length_q > radius**2 and self._query_visibility_recursive(q1, q2, radius, node.get_left()) and self._query_visibility_recursive(q1, q2, radius, node.get_right()))
