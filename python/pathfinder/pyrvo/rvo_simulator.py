from typing import Tuple, List, Optional
from pathfinder.pyrvo.kd_tree import KDTree
from pathfinder.pyrvo.agent import Agent, linear_program2, linear_program3
from pathfinder.pyrvo.obstacle import Obstacle

class RVOSimulator:
    def __init__(self, neighbor_dist: float,
                 max_neighbors: int,
                 time_horizon: float,
                 time_horizon_obst: float,
                 radius: float,
                 max_speed: float):
        self._kd_tree = KDTree(self)
        self._default_agent = Agent(self, -1, None, (0.0, 0.0), (0.0, 0.0),
                                    neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed)
        # set default agent parameters
        # self._default_agent.set_parameters(max_neighbors, max_speed, neighbor_dist, radius, time_horizon, time_horizon_obst, (0.0, 0.0))
        self._agents: List[Agent] = []  # set agents empty array
        self._obstacles: List[Obstacle] = []  # also for obstacles
        self._force_update_agents = True

    def add_agent(self, position: Tuple[float, float],
                  radius: Optional[float] = None,
                  velocity: Tuple[float, float] = (0.0, 0.0),
                  neighbor_dist: Optional[float] = None,
                  max_neighbors: Optional[int] = None,
                  time_horizon: Optional[float] = None,
                  time_horizon_obst: Optional[float] = None,
                  max_speed: Optional[float] = None) -> int:
        agent = Agent(self, len(self._agents), self._default_agent, position, velocity, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed)
        self._agents.append(agent)
        self._force_update_agents = True
        return len(self._agents) - 1

    def delete_agents(self, agent_indexes: List[int]):
        for i in range(len(agent_indexes)):
            agent_index = agent_indexes[len(agent_indexes) - 1 - i]
            self._agents.pop(agent_index)
        self._force_update_agents = True

    def get_agent(self, agent_index: int) -> Agent:
        return self._agents[agent_index]

    def get_all_agents(self) -> List[Agent]:
        return self._agents

    def add_obstacle(self, vertices: List[Tuple[float, float]]) -> int:
        if len(vertices) >= 2:
            obstacle_no: int = len(self._obstacles)
            for i in range(len(vertices)):
                obstacle = Obstacle()
                obstacle.set_point(vertices[i])
                if i != 0:
                    obstacle.set_prev_obstacle(self._obstacles[-1])
                    obstacle.get_prev_obstacle().set_next_obstacle(obstacle)

                if i == len(vertices) - 1:
                    obstacle.set_next_obstacle(self._obstacles[obstacle_no])
                    obstacle.get_next_obstacle().set_prev_obstacle(obstacle)

                obstacle.set_unit_dir(vertices[0 if (i == len(vertices) - 1) else i + 1], vertices[i])
                if len(vertices) == 2:
                    obstacle.set_is_convex_value(True)
                else:
                    obstacle.set_is_convex(vertices[(len(vertices) - 1 if i == 0 else i - 1)], vertices[i], vertices[0 if (i == len(vertices) - 1) else i + 1])

                obstacle.set_id(len(self._obstacles))
                self._obstacles.append(obstacle)
            return obstacle_no
                
        else:
            return -1

    def compute_agent_obstacles_neighbors(self, agent, range_square: float):
        self._kd_tree.compute_obstacle_neighbors(agent, range_square)

    def compute_agent_agents_neighbors(self, agent, range_square: float):
        self._kd_tree.compute_agent_neighbors(agent, range_square)

    def add_obstacle_object(self, obstacle: Obstacle):
        self._obstacles.append(obstacle)

    def do_step(self, delta_time: float, move_agents: bool = True):
        self._kd_tree.build_agent_tree(self._force_update_agents)
        self._force_update_agents = False
        for i in range(len(self._agents)):
            self._agents[i].compute_neighbors()
            self._agents[i].compute_new_velocity(delta_time)

        for i in range(len(self._agents)):
            self._agents[i].update(delta_time, move_agents)

    def get_agent_agent_neighbor(self, agent_index: int, neighbor_index: int) -> int:
        return self._agents[agent_index].get_agent_neighbors()[neighbor_index][1].get_id()

    def get_agent_max_neighbors(self, agent_index: int) -> int:
        return self._agents[agent_index].get_max_neighbors()

    def get_agent_max_speed(self, agent_index: int) -> float:
        return self._agents[agent_index].get_max_speed()

    def get_agent_neighbor_dist(self, agent_index: int) -> float:
        return self._agents[agent_index].get_neighbor_distance()

    def get_agent_num_agent_neighbors(self, agent_index: int) -> int:
        return len(self._agents[agent_index].get_agent_neighbors())

    def get_agent_num_obstacle_neighbors(self, agent_index) -> int:
        return len(self._agents[agent_index].get_obstacle_neighbors())

    def get_agent_obstacle_neighbor(self, agent_index: int, neighbor_index: int):
        return self._agents[agent_index].get_obstacle_neighbors()[neighbor_index][1].get_id()

    def get_agent_position(self, agent_index: int) -> Tuple[float, float]:
        return self._agents[agent_index].get_position()

    def get_agent_pref_velocity(self, agent_index: int) -> Optional[Tuple[float, float]]:
        return self._agents[agent_index].get_pref_velocity()

    def get_agent_radius(self, agent_index: int) -> float:
        return self._agents[agent_index].get_radius()

    def get_agent_time_horizon(self, agent_index: int) -> float:
        return self._agents[agent_index].get_time_horizon()

    def get_agent_time_horizon_obst(self, agent_index: int) -> float:
        return self._agents[agent_index].get_time_horizon_obst()

    def get_agent_velocity(self, agent_index: int) -> Tuple[float, float]:
        return self._agents[agent_index].get_velocity()

    def get_agents_count(self) -> int:
        return len(self._agents)

    def get_obstacles_count(self) -> int:
        return len(self._obstacles)

    def get_obstacle(self, obst_index: int) -> Obstacle:
        return self._obstacles[obst_index]

    def get_obstacle_vertex(self, vertex_index: int) -> Tuple[float, float]:
        return self._obstacles[vertex_index].get_point()

    def get_next_obstacle_vertex_index(self, vertex_index: int) -> int:
        return self._obstacles[vertex_index].get_next_obstacle().get_id()

    def get_prev_obstacle_vertex_index(self, vertex_index: int) -> int:
        return self._obstacles[vertex_index].get_prev_obstacle().get_id()

    def process_obstacles(self):
        self._kd_tree.build_obstacle_tree()

    def query_visibility(self, start: Tuple[float, float], end: Tuple[float, float], radius: float = 0.0) -> bool:
        return self._kd_tree.query_visibility(start, end, radius)

    def set_agent_defaults(self,
                           neighbor_dist: float,
                           max_neighbors: int,
                           time_horizon: float,
                           time_horizon_obst: float,
                           radius: float,
                           max_speed: float):
        self._default_agent.set_parameters(max_neighbors, max_speed, neighbor_dist, radius, time_horizon, time_horizon_obst, (0.0, 0.0))

    def set_agent_max_neighbors(self, agent_index: int, max_neighbors: int):
        self._agents[agent_index].set_max_neighbors(max_neighbors)

    def set_agent_max_speed(self, agent_index: int, max_speed: float):
        self._agents[agent_index].set_max_speed(max_speed)

    def set_agent_neighbor_dist(self, agent_index: int, neighbor_dist: float):
        self._agents[agent_index].set_neighbor_distance(neighbor_dist)

    def set_agent_position(self, agent_index: int, position: Tuple[float, float]):
        self._agents[agent_index].set_position(position)

    def set_agent_pref_velocity(self, agent_index: int, velocity: Tuple[float, float]):
        self._agents[agent_index].set_pref_velocity(velocity)

    def set_agent_radius(self, agent_index: int, radius: float):
        self._agents[agent_index].set_radius(radius)

    def set_agent_time_horizon(self, agent_index: int, time_horizon: float):
        self._agents[agent_index].set_time_horizon(time_horizon)

    def set_agent_time_horizon_obst(self, agent_index: int, time_horizon_obst: float):
        self._agents[agent_index].set_time_horizon_obst(time_horizon_obst)

    def set_agent_velocity(self, agent_index: int, velocity: Tuple[float, float]):
        self._agents[agent_index].set_velocity(velocity)
