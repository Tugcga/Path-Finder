import pathfinder.pyrvo as rvo
from pathfinder import PathFinder
import math
import random
import time
from typing import Tuple


def example_simple_rvo():
    # create simulator
    simulator = rvo.create_simulator(neighbor_dist=1.0,
                                     max_neighbors=5,
                                     time_horizon=1.0,
                                     time_horizon_obst=1.0,
                                     radius=0.5,
                                     max_speed=5.0)
    # add several agents to the simulation
    agents_count: int = 100
    for a in range(agents_count):
        # generate random position
        position = (random.uniform(-5.0, 5.0), random.uniform(-5.0, 5.0))
        # add agent at this position
        rvo.add_agent(simulator, position)

    # define the target for all agents
    target: Tuple[float, float] = (0.0, 0.0)

    # start simulation
    start_time: float = time.time()
    simulation_steps: int = 1
    for step in range(simulation_steps):
        # compute prefered velocity (to the target) for each agent
        for a in range(agents_count):
            # get actual position
            pos = rvo.get_agent_position(simulator, a)
            # compute move vector
            to_vector: Tuple[float, float] = (target[0] - pos[0],
                                              target[1] - pos[1])
            to_length: float = math.sqrt(to_vector[0]**2 + to_vector[1]**2)
            # for long vector normalize it, for short one use it as is
            # move speed is a length of the prefered velocity vector
            if to_length > 1.0:
                to_vector = (to_vector[0] / to_length, 
                             to_vector[1] / to_length)
            # set prefered velocity
            rvo.set_agent_pref_velocity(simulator, a, to_vector)
        # simulate one step, 0.1 is a delta time
        rvo.simulate(simulator, 0.1)

    finish_time: float = time.time()
    print("total time for simulation", agents_count, "agents and", 
          simulation_steps, "steps:", finish_time - start_time, "seconds")
    # for python version: 100 agents and 1000 steps: 10.4 seconds
    # for binary version: 100 agents and 1000 steps: 0.32 seconds, 1000 / 1000: 3.01 seconds, 100 000 / 1: 0.51 sec


def example_pathfinder():
    # create navmesh as simple square
    vertices = [(-4.0, 0.0, -4.0), 
                (4.0, 0.0, -4.0), 
                (4.0, 0.0, 4.0), 
                (-4.0, 0.0, 4.0)]
    polygons = [[0, 1, 2, 3]]
    pf = PathFinder(vertices, polygons)

    # add agent to the pathfinder at (2.0, 2.0) position, 0.5 radius and 2.0 speed
    pf.add_agent((2.0, 0.0, 2.0), 0.5, 2.0)  # y-position will be ignored in simulation, but will be used in intermediate path findings

    # set destination point
    pf.set_agent_destination(0, (-2.0, 0.0, -2.0))

    # call update several times
    steps_count: int = 5
    for step in range(steps_count):
        pf.update()
        time.sleep(0.1)
    # output current agent position
    print(pf.get_agent_position(0))


if __name__ == "__main__":
    example_pathfinder()
