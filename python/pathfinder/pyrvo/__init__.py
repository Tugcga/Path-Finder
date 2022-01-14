IMPORT_BINARY = False
if IMPORT_BINARY:
    try:
        import pathfinder.pyrvo.rvo2 as rvo  # type: ignore
    except Exception as e:
        print("Fails to load binary RVO2 library, switch to Python version")
        IMPORT_BINARY = False
if IMPORT_BINARY is False:
    from pathfinder.pyrvo.rvo_simulator import RVOSimulator
from typing import Tuple, List, Optional

# -------------------------------------
# processors
# -------------------------------------

def create_simulator(neighbor_dist: float = 1.5, 
                     max_neighbors: int = 5,
                     time_horizon: float = 1.5, 
                     time_horizon_obst: float = 2.0,
                     radius: float = 0.4,
                     max_speed: float = 2.0):
    if IMPORT_BINARY:
        return rvo.PyRVOSimulator(0.0, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed)
    else:
        return RVOSimulator(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed)

def add_agent_default(simulator, 
                      position: Tuple[float, float]) -> int:
    '''
    return index of the agent
    '''
    if IMPORT_BINARY:
        return simulator.addAgent(position)
    else:
        return simulator.add_agent(position)

def add_agent(simulator,
              position: Tuple[float, float],
              neighbor_dist: Optional[float] = None,
              max_neighbors: Optional[int] = None,
              time_horizon: Optional[float] = None,
              time_horizon_obst: Optional[float] = None,
              radius: Optional[float] = None,
              max_speed: Optional[float] = None,
              velocity: Tuple[float, float] = (0.0, 0.0)):
    '''
    return index of the agent
    '''
    if IMPORT_BINARY:
        return simulator.addAgent(position, 
                                  neighbor_dist if neighbor_dist is not None else 1.5, 
                                  max_neighbors if max_neighbors is not None else 5, 
                                  time_horizon if time_horizon is not None else 1.5, 
                                  time_horizon_obst if time_horizon_obst is not None else 2.0, 
                                  radius if radius is not None else 0.4, 
                                  max_speed if max_speed is not None else 2.0, 
                                  velocity)
    else:
        return simulator.add_agent(position, radius, velocity, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, max_speed)

def delete_agent(simulator, agent_indexes: List[int]):
    '''remove agent from the simulation

    agent_indexs is ordered array
    '''
    if IMPORT_BINARY:
        raise NotImplementedError("delete agent is not implemented in binary RVO2 library")
    else:
        simulator.delete_agents(agent_indexes)

def add_obstacle(simulator, 
                 vertices: List[Tuple[float, float]]) -> int:
    '''
    return index of the first obstacle object
    '''
    if IMPORT_BINARY:
        return simulator.addObstacle(vertices)
    else:
        return simulator.add_obstacle(vertices)

def process_obstacles(simulator):
    if IMPORT_BINARY:
        simulator.processObstacles()
    else:
        simulator.process_obstacles()

def query_visibility(simulator, point_01: Tuple[float, float], point_02: Tuple[float, float]) -> bool:
    if IMPORT_BINARY:
        return simulator.queryVisibility(point_01, point_02)
    else:
        return simulator.query_visibility(point_01, point_02)

def simulate(simulator,
             delta_time: float,
             move_agents: bool = True):
    '''make one step of the simulation
    '''
    if IMPORT_BINARY:
        simulator.setTimeStep(delta_time)
        simulator.doStep()
    else:
        simulator.do_step(delta_time, move_agents)

# -------------------------------------
# get agent methods
# -------------------------------------
def get_agent_agent_neighbor(simulator, 
                             agent_index: int, 
                             neighbor_index: int) -> int:
    if IMPORT_BINARY:
        return simulator.getAgentAgentNeighbor(agent_index, neighbor_index)
    else:
        return simulator.get_agent_agent_neighbor(agent_index, neighbor_index)

def get_agent_max_neighbors(simulator, 
                            agent_index: int) -> int:
    if IMPORT_BINARY:
        return simulator.getAgentMaxNeighbors(agent_index)
    else:
        return simulator.get_agent_max_neighbors(agent_index)

def get_agent_max_speed(simulator, 
                        agent_index: int) -> float:
    if IMPORT_BINARY:
        return simulator.getAgentMaxSpeed(agent_index)
    else:
        return simulator.get_agent_max_speed(agent_index)

def get_agent_neighbor_dist(simulator, 
                            agent_index: int) -> float:
    if IMPORT_BINARY:
        return simulator.getAgentNeighborDist(agent_index)
    else:
        return simulator.get_agent_neighbor_dist(agent_index)

def get_agent_num_agent_neighbors(simulator, 
                                  agent_index: int) -> int:
    if IMPORT_BINARY:
        return simulator.getAgentNumAgentNeighbors(agent_index)
    else:
        return simulator.get_agent_num_agent_neighbors(agent_index)

def get_agent_num_obstacle_neighbors(simulator, 
                                     agent_index: int) -> int:
    if IMPORT_BINARY:
        return simulator.getAgentNumObstacleNeighbors(agent_index)
    else:
        return simulator.get_agent_num_obstacle_neighbors(agent_index)

def get_agent_obstacle_neighbor(simulator, 
                                agent_index: int, 
                                neighbor_index: int) -> int:
    if IMPORT_BINARY:
        return simulator.getAgentObstacleNeighbor(agent_index, neighbor_index)
    else:
        return simulator.get_agent_obstacle_neighbor(agent_index, neighbor_index)

def get_agent_position(simulator,
                       agent_index: int) -> Tuple[float, float]:
    if IMPORT_BINARY:
        return simulator.getAgentPosition(agent_index)
    return simulator.get_agent_position(agent_index)

def get_agent_pref_velocity(simulator,
                            agent_index: int) -> Tuple[float, float]:
    if IMPORT_BINARY:
        return simulator.getAgentPrefVelocity(agent_index)
    return simulator.get_agent_pref_velocity(agent_index)

def get_agent_radius(simulator,
                     agent_index: int) -> float:
    if IMPORT_BINARY:
        return simulator.getAgentRadius(agent_index)
    else:
        return simulator.get_agent_radius(agent_index)

def get_agent_time_horizon(simulator,
                           agent_index: int) -> float:
    if IMPORT_BINARY:
        return simulator.getAgentTimeHorizon(agent_index)
    else:
        return simulator.get_agent_time_horizon(agent_index)

def get_agent_time_horizon_obst(simulator,
                                agent_index: int) -> float:
    if IMPORT_BINARY:
        return simulator.getAgentTimeHorizonObst(agent_index)
    else:
        return simulator.get_agent_time_horizon_obst(agent_index)

def get_agent_velocity(simulator,
                       agent_index: int) -> Tuple[float, float]:
    if IMPORT_BINARY:
        return simulator.getAgentVelocity(agent_index)
    else:
        return simulator.get_agent_velocity(agent_index)

def get_agents_count(simulator) -> int:
    if IMPORT_BINARY:
        return simulator.getNumAgents()
    else:
        return simulator.get_agents_count()

# -------------------------------------
# set agent methods
# -------------------------------------

def set_agent_defaults(simulator,
                     neighbor_dist: float,
                     max_neighbors: int,
                     time_horizon: float,
                     time_horizon_obst: float,
                     radius: float,
                     max_speed: float):
    if IMPORT_BINARY:
        simulator.setAgentDefaults(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed)
    else:
        simulator.set_agent_defaults(neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed)

def set_agent_max_neighbors(simulator, 
                            agent_index: int,
                            max_neighbors: int):
    if IMPORT_BINARY:
        simulator.setAgentMaxNeighbors(agent_index, max_neighbors)
    else:
        simulator.set_agent_max_neighbors(agent_index, max_neighbors)

def set_agent_max_speed(simulator, 
                        agent_index: int,
                        max_speed: float):
    if IMPORT_BINARY:
        simulator.setAgentMaxSpeed(agent_index, max_speed)
    else:
        simulator.set_agent_max_speed(agent_index, max_speed)

def set_agent_neighbor_dist(simulator, 
                            agent_index: int,
                            neighbor_dist: float):
    if IMPORT_BINARY:
        simulator.setAgentNeighborDist(agent_index, neighbor_dist)
    else:
        simulator.set_agent_neighbor_dist(agent_index, neighbor_dist)

def set_agent_position(simulator, agent_index: int, position: Tuple[float, float]):
    if IMPORT_BINARY:
        simulator.setAgentPosition(agent_index, position)
    else:
        simulator.set_agent_position(agent_index, position)

def set_agent_pref_velocity(simulator, 
                            agent_index: int, 
                            velocity: Tuple[float, float]):
    '''set prefered velocity for the agent with a given index
    '''
    if IMPORT_BINARY:
        simulator.setAgentPrefVelocity(agent_index, velocity)
    else:
        simulator.set_agent_pref_velocity(agent_index, velocity)

def set_agent_radius(simulator,
                     agent_index: int,
                     radius: float):
    if IMPORT_BINARY:
        simulator.setAgentRadius(agent_index, radius)
    else:
        simulator.set_agent_radius(agent_index, radius)

def set_agent_time_horizon(simulator,
                           agent_index: int,
                           time_horizon: float):
    if IMPORT_BINARY:
        simulator.setAgentTimeHorizon(agent_index, time_horizon)
    else:
        simulator.set_agent_time_horizon(agent_index, time_horizon)

def set_agent_time_horizon_obst(simulator,
                                agent_index: int,
                                time_horizon_obst):
    if IMPORT_BINARY:
        simulator.setAgentTimeHorizonObst(agent_index, time_horizon_obst)
    else:
        simulator.set_agent_time_horizon_obst(agent_index, time_horizon_obst)

def set_agent_velocity(simulator,
                       agent_index: int,
                       velocity: Tuple[float, float]):
    if IMPORT_BINARY:
        simulator.setAgentVelocity(agent_index, velocity)
    else:
        simulator.set_agent_velocity(agent_index, velocity)
