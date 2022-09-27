from typing import Tuple, List, Optional
import math
import time
import struct
from pathfinder.navmesh import Navmesh
import pathfinder.pyrvo as rvo


def get_vector_length_squared(vector_2d: Tuple[float, float]):
    return vector_2d[0]**2 + vector_2d[1]**2


def get_vector_length(vector_2d: Tuple[float, float]):
    return math.sqrt(get_vector_length_squared(vector_2d))


def get_unit_vector(vector_2d: Tuple[float, float]):
    l: float = get_vector_length(vector_2d)
    if l < 0.00001:
        return (0.0, 0.0)
    else:
        return (vector_2d[0] / l, vector_2d[1] / l)


def read_from_binary(file_path: str) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
    '''Read input binary file with navigation mesh polygonal data and return arrays with vertices and polygons

    The format of a binary file is very simple
    It contains three array, splitted by 64 bit infinite float
    The first array contains vertex positions as array of 64 bit floats
    The second array contains 32 bit integers, which corresponds to polygon vertex indices
    The third array contains 32 bit integers, which define polygon sizes
    The file ends by 32 bits infinite float

    Byte order is big-endian
    '''
    vertices: List[Tuple[float, float, float]] = []
    polygons: List[List[int]] = []
    with open(file_path, "rb") as file:
        is_finish_vertices: bool = False
        x: float = 0.0
        y: float = 0.0
        index = 0
        while not is_finish_vertices:
            bs = file.read(4)  # read by 8 bytes
            value = struct.unpack(">f", bs)[0]
            if value == float("inf"):
                is_finish_vertices = True
            else:
                index += 1
                if index == 1:
                    x = value
                elif index == 2:
                    y = value
                elif index == 3:
                    index = 0
                    vertices.append((x, y, value))

        is_finish_polygons: bool = False
        polygons_indexes: List[int] = []  # store here flat array of polygon indexes
        while not is_finish_polygons:
            bs = file.read(4)  # read by 4 bytes, because file store integers as 32 bits
            value = struct.unpack(">i", bs)[0]
            value_float = struct.unpack(">f", bs)[0]
            if value_float == float("inf"):
                is_finish_polygons = True
            else:
                polygons_indexes.append(value)

        is_finish_sizes: bool = False
        index = 0
        while not is_finish_sizes:
            bs = file.read(4)
            value = struct.unpack(">i", bs)[0]
            value_float = struct.unpack(">f", bs)[0]
            if value_float == float("inf"):
                is_finish_sizes = True
            else:
                polygons.append(polygons_indexes[index:index+value])
                index += value

    return (vertices, polygons)


def read_from_text(file_path: str) -> Tuple[List[Tuple[float, float, float]], List[List[int]]]:
    '''Read input text file with navigation mesh polygonal data and return arrays with vertices and polygons
    '''
    with open(file_path, "r") as file:
        file_text = file.read()
        lines = file_text.split("\n")
        if len(lines) == 3:  # polygons data
            vertices_raw = [float(v) for v in lines[0].split(" ")]
            polygons_raw = [int(v) for v in lines[1].split(" ")]
            sizes = [int(v) for v in lines[2].split(" ")]

            verts_count = len(vertices_raw) // 3
            vertices = []
            for i in range(verts_count):
                vertices.append((vertices_raw[3*i], vertices_raw[3*i + 1], vertices_raw[3*i + 2]))

            i = 0
            polygons = []
            for s in sizes:
                polygon = []
                for j in range(s):
                    polygon.append(polygons_raw[i])
                    i += 1
                polygons.append(polygon)
            return (vertices, polygons)
    return ([], [])


class PathFinder(object):
    '''Main class for pathfinding process
    '''
    def __init__(self, 
                 vertices: Optional[List[Tuple[float, float, float]]] = None, 
                 polygons: Optional[List[List[int]]] = None,
                 neighbor_dist: float = 1.5,
                 max_neighbors: int = 5,
                 time_horizon: float = 1.5,
                 time_horizon_obst: float = 2.0,
                 max_speed: float = 10.0,
                 agent_radius: float = 0.2,
                 update_path_find: float = 1.0,
                 continuous_moving: bool = False,
                 move_agents: bool = True):
        '''Init pathfinder object by setting vertices and polygons of the navmesh
        If vertices or polygons are not defined, then navigation mesh is not created. In this case you can only simulate RVO on infinite plane without obstacles

        Input:
            vertices - array of the form [(x1, y1, z1), (x2, y2, z1), ...]
            polygons - array of the form [p1, p2, ...], where each pi is array [i1, i2, ..., in], ij - indexes of vertices
                       the sequence [i1, i2, ..., in] should be in clockwise direction from the main point of view
                       this is importatnt for search path algorithm
                       if polygon orientation will be conter-clockwise, then the path will be incorrect
            agent_radius - default radius of an agent. We use this value for constructing rvo obstacles. Real agents can be created with other radius value
            update_path_find - how often refind shortest path in navmesh for active agents (mesured in seconds)
            continuous_moving - if True then agents always move to destination point
            move_agents - if True then move agents in RVO algorithm, if False then only calulate velocities

        Example of the simple square grid with two 4-sided polygons
            vertices = [(1.0, 0.0, 1.0), (-1.0, 0.0, 1.0), (-1.0, 0.0, -1.0), (1.0, 0.0, -1.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0)]
            polygons = [[0, 3, 5, 4], [4, 5, 2, 1]]
        '''
        self._navmesh_boundary: List[List[List[Tuple[int, int]]]] = []  # each value in the array corresponds to one group
        if vertices is None or polygons is None:
            self._navmesh = None
        else:
            self._navmesh = Navmesh(vertices, polygons)
            # calculate boundary for rvo obstacles
            # we should build boundary for each group
            # boundary for a group is array of chains (without final edge)
            # each chain is array of int-pairs
            groups_count = self._navmesh.get_groups_count()
            for g_index in range(groups_count):
                group_polygons = self._navmesh.get_group_polygons(g_index)
                # collect all edges
                all_edges = []
                for p in group_polygons:
                    # p is an array of polygon vertex indices
                    for i in range(len(p)):
                        all_edges.append((p[i], p[i + 1 if i + 1 < len(p) else 0]))
                # next we should find boundary edges
                boundary_edges = []
                for e in all_edges:
                    if (e[1], e[0]) not in all_edges:
                        boundary_edges.append(e)
                # next we should order edges in cycles
                boundary_chains = []
                current_chain = [boundary_edges.pop(-1)]
                while len(boundary_edges) > 0:
                    # next find the edge with the same start as the end of e
                    e = current_chain[-1]
                    is_find = False
                    i = 0
                    while not is_find:
                        f = boundary_edges[i]
                        if f[0] == e[1]:
                            is_find = True
                        else:
                            i += 1
                        if i >= len(boundary_edges):
                            is_find = True
                    if i >= len(boundary_edges):
                        # we fails to find the next edge
                        # drop this chain
                        current_chain = [boundary_edges.pop(-1)]
                    else:
                        f = boundary_edges.pop(i)
                        if f[1] == current_chain[0][0]:
                            # this is the last edge in the cycles of the chain, does not add it
                            boundary_chains.append(current_chain)
                            if len(boundary_edges) > 0:
                                current_chain = [boundary_edges.pop(-1)]
                        else:
                            current_chain.append(f)
                self._navmesh_boundary.append(boundary_chains)
        # next create rvo simulators
        # memorize default agent parameters
        self._neighbor_dist: float = neighbor_dist
        self._max_neighbors: int = max_neighbors
        self._time_horizon: float = time_horizon
        self._time_horizon_obst: float = time_horizon_obst
        self._max_speed: float = max_speed
        self._agent_radius: float = agent_radius
        self._update_path_find: float = update_path_find
        self._continuous_moving: bool = continuous_moving
        self._move_agents: bool = move_agents
        self._last_path_find_update: float = time.time()
        # create separate simulator for each group
        self._simulators = []
        self._groups_count = self._navmesh.get_groups_count() if self._navmesh is not None else 1
        for g in range(self._groups_count):
            group_simulator = rvo.create_simulator(neighbor_dist=self._neighbor_dist, 
                                                   max_neighbors=self._max_neighbors,
                                                   time_horizon=self._time_horizon, 
                                                   time_horizon_obst=self._time_horizon_obst,
                                                   radius=self._agent_radius,
                                                   max_speed=self._max_speed)
            self._simulators.append(group_simulator)

        # calculate boundary for rvo
        # we should shift all edges of the navmesh boundary into agent radius value
        shift_value: float = 1.0 * agent_radius
        self._obstacles: List[List[Tuple[float, float]]] = []
        if vertices is not None:
            for group_index in range(len(self._navmesh_boundary)):
                boundary: List[List[Tuple[int, int]]] = self._navmesh_boundary[group_index]
                for chain in boundary:
                    chain_vertices = []
                    for edge in chain:
                        chain_vertices.append((vertices[edge[0]][0], vertices[edge[0]][2]))  # ignore y-coordinate
                    chain_vertices.append((vertices[chain[-1][1]][0], vertices[chain[-1][1]][2]))

                    shifted_chain = []
                    for i in range(len(chain) + 1):
                        pre_point = chain_vertices[i - 1 if i - 1 >= 0 else -1]
                        point = chain_vertices[i]
                        post_point = chain_vertices[i + 1 if i + 1 < len(chain_vertices) else 0]
                        a1 = (point[0] - pre_point[0], point[1] - pre_point[1])
                        a2 = (point[0] - post_point[0], point[1] - post_point[1])
                        l1 = math.sqrt(a1[0]**2 + a1[1]**2)
                        l2 = math.sqrt(a2[0]**2 + a2[1]**2)
                        a1 = (a1[0] / l1, a1[1] / l1)
                        a2 = (a2[0] / l2, a2[1] / l2)
                        n1 = (-a1[1], a1[0])
                        n2 = (a2[1], -a2[0])
                        # calculate new position as intersection point between two lines (original edges, shifted along normals)
                        if abs(a1[1] * a2[0] - a1[0] * a2[1]) < 0.0001:
                            # edges are parallel, simply shift along n1
                            new_point = (point[0] + n1[0]*shift_value, point[1] + n1[1]*shift_value)
                        else:
                            t = (a2[0]*(post_point[1] + n2[1]*shift_value - pre_point[1] - n1[1]*shift_value) + a2[1]*(pre_point[0] + n1[0]*shift_value - post_point[0] - n2[0]*shift_value)) / (a1[1] * a2[0] - a1[0] * a2[1])
                            new_point = (pre_point[0] + n1[0]*shift_value + a1[0] * t, pre_point[1] + n1[1]*shift_value + a1[1] * t)
                        shifted_chain.append(new_point)
                    # set obstacle
                    rvo.add_obstacle(self._simulators[group_index], shifted_chain)
                    self._obstacles.append(shifted_chain)
                rvo.process_obstacles(self._simulators[group_index])

        self._agent_id = 0  # use this value for adding the new agent
        self._agents_speed: List[float] = []
        self._agents_activity: List[bool] = []  # set True for agent if it should move to target
        self._agents_targets: List[List[Tuple[float, float]]] = []  # for each agent we set array of 2d-positions
        self._agents_path: List[List[Tuple[float, float, float]]] = []  # store here apth for each agent
        self._agents_target_index: List[int] = []  # store here index to the actual target for each agent
        self._agents_target_direction: List[Tuple[float, float]] = []  # store here directions to the targets, calculated when we change it
        self._agents_height: List[List[float]] = []  # store here y-height of the path points
        self._agents_group: List[int] = []  # store here group of an each agent, position of the value in the array is agent index in total list of ids
        self._agents_group_id: List[List[int]] = []  # for each group store ids of agents in the current simulator
        for g in range(self._groups_count):
            self._agents_group_id.append([])  # init by emty arrays
        self._agents_id: List[int] = []  # plain list of ids of all agents, index of the id allows to find agent data in other arrays
        self._agents_to_delete: List[int] = []  # store here agent ids we need to delete before update step
        self._last_update_time = time.time()

    def add_agent(self, position: Tuple[float, float, float], radius: float, speed: float) -> int:
        '''return agent id

        -1 - fails to add the agent
        '''
        is_add = True
        add_group = 0
        if self._navmesh is not None:
            node = self._navmesh.sample_polygon(position)
            if node is None:
                is_add = False
            else:
                # get valid group of the polygon
                add_group = node.get_group()
        if is_add:
            add_position = (position[0], position[2])  # use only x and z coordinate
            rvo.add_agent(self._simulators[add_group],
                          position=add_position,
                          neighbor_dist=self._neighbor_dist,
                          max_neighbors=self._max_neighbors,
                          time_horizon=self._time_horizon,
                          time_horizon_obst=self._time_horizon_obst,
                          radius=radius,
                          max_speed=speed)
            self._agents_speed.append(speed)
            self._agents_activity.append(False)
            self._agents_targets.append([])
            self._agents_path.append([])  # add for the given agent empty path
            self._agents_target_index.append(0)
            self._agents_target_direction.append((0.0, 0.0))
            self._agents_height.append([])
            self._agents_group.append(add_group)
            self._agents_group_id[add_group].append(self._agent_id)

            self._agents_id.append(self._agent_id)
            self._agent_id += 1
            return self._agent_id - 1
        return -1

    def delete_agent(self, agent_id: int):
        self._agents_to_delete.append(agent_id)

    def get_obstacles_points(self) -> List[List[Tuple[float, float]]]:
        return self._obstacles

    def _to_direction(self, from_point: Tuple[float, float], to_point: Tuple[float, float]) -> Tuple[float, float]:
        '''create direction unit vector from one point to the other in 2d
        '''
        v = [to_point[0] - from_point[0], to_point[1] - from_point[1]]
        l = math.sqrt(v[0]**2 + v[1]**2)
        if l < 0.00001:
            return (0.0, 0.0)
        else:
            return (v[0] / l, v[1] / l)

    def set_agent_destination(self, agent_id, position: Tuple[float, float, float]):
        '''Calculate the path from current agent position to the destination position
        '''
        p = self.get_agent_position(agent_id)
        a_path = self.search_path((p[0], 0.0, p[1]), position)
        self._set_agent_path(agent_id, a_path)  # set raw 3-float tuples path

    def _set_agent_path(self, agent_id: int, path: List[Tuple[float, float, float]]):
        agent_index = self._get_agent_inner_index(agent_id)
        if agent_index  > -1:
            if len(path) > 0:
                if len(path) == 1:
                    path.append(path[0])
                self._agents_path[agent_index] = path
                self._agents_targets[agent_index] = [(v[0], v[2]) for v in path]
                self._agents_target_index[agent_index] = 1  # we start with the second value, because the first value is start point
                # get current agent position
                position = self.get_agent_position(agent_id, agent_index)
                target = self._agents_targets[agent_index][1]
                # set direction
                self._agents_target_direction[agent_index] = self._to_direction(position, target)
                self._agents_activity[agent_index] = True
                self._agents_height[agent_index] = []
                for p in path:
                    self._agents_height[agent_index].append(p[1])
            else:
                # the input path is empty, nothing to do
                pass
        else:
            raise IndexError("there are no agent with id " + str(agent_id))

    def update_time(self):
        self._last_update_time = time.time()

    def update(self):
        t = time.time()
        update_path = True if t - self._last_path_find_update > self._update_path_find else False
        if update_path:
            self._last_path_find_update = t
        delta_time = t - self._last_update_time

        # delete agents before update step
        if len(self._agents_to_delete) > 0:
            indexes_in_groups = []  # store here indexes in each group
            inner_indexes = []
            for group_index in range(self._groups_count):
                indexes_in_groups.append([])
            for agent_id in self._agents_to_delete:
                agent_inner_index = self._get_agent_inner_index(agent_id)
                inner_indexes.append(agent_inner_index)
                if agent_inner_index > -1:
                    group_index = self._agents_group[agent_inner_index]
                    # call delete from simulation
                    agent_in_group_index = self._get_agent_group_index(agent_id, self._agents_group_id[group_index])
                    indexes_in_groups[group_index].append(agent_in_group_index)
            for group_indexes in indexes_in_groups:
                group_indexes.sort()
                rvo.delete_agent(self._simulators[group_index], group_indexes)
            # and now we a ready to delete data from arrays
            inner_indexes.sort()
            for i in range(len(inner_indexes)):
                agent_inner_index = inner_indexes[len(inner_indexes) - 1 - i]
                agent_id = self._agents_id[agent_inner_index]
                group_index = self._agents_group[agent_inner_index]
                agent_in_group_index = self._get_agent_group_index(agent_id, self._agents_group_id[group_index])
                # delete
                self._agents_height.pop(agent_inner_index)
                self._agents_target_direction.pop(agent_inner_index)
                self._agents_target_index.pop(agent_inner_index)
                self._agents_targets.pop(agent_inner_index)
                self._agents_path.pop(agent_inner_index)
                self._agents_activity.pop(agent_inner_index)
                self._agents_speed.pop(agent_inner_index)
                self._agents_group_id[self._agents_group[agent_inner_index]].pop(agent_in_group_index)
                self._agents_group.pop(agent_inner_index)
                self._agents_id.pop(agent_inner_index)
            self._agents_to_delete = []

        for agent_inner_index, agent_id in enumerate(self._agents_id):
            should_deactivate: bool = False
            group_index = self._agents_group[agent_inner_index]
            sim = self._simulators[group_index]
            agent_index = self._get_agent_group_index(agent_id, self._agents_group_id[group_index])  # index of the agent in the simulator
            if self._agents_activity[agent_inner_index]:
                current_position = rvo.get_agent_position(sim, agent_index)
                # calculate velocity vector
                agent_target_index = self._agents_target_index[agent_inner_index]
                agent_targets_count = len(self._agents_targets[agent_inner_index])
                target = self._agents_targets[agent_inner_index][agent_target_index]  # get path for the agent and select proper position in the path
                to_vector = (target[0] - current_position[0], target[1] - current_position[1])
                distance_to_target = math.sqrt(to_vector[0]**2 + to_vector[1]**2)
                a_speed = self._agents_speed[agent_inner_index]
                # check is target is a finish point and agent close to it
                if agent_target_index == agent_targets_count - 1 and distance_to_target < delta_time * a_speed:
                    # set last velocity for the agent and deactivate it
                    a_velocity = get_unit_vector(to_vector)
                    last_speed = distance_to_target / delta_time
                    rvo.set_agent_pref_velocity(sim, agent_index, (a_velocity[0] * last_speed, a_velocity[1] * last_speed))
                    should_deactivate = True
                else:
                    local_dir = self._to_direction(current_position, target)
                    start_dir = self._agents_target_direction[agent_inner_index]
                    d = local_dir[0] * start_dir[0] + local_dir[1] * start_dir[1]
                    if d < 0.0:
                        # the agent go over the target
                        if agent_target_index  < agent_targets_count - 1:
                            # there are other targets in the path
                            # try to switch to the enxt target point
                            next_target: Tuple[float, float] = self._agents_targets[agent_inner_index][agent_target_index + 1]
                            is_next_visible: bool = rvo.query_visibility(sim, current_position, next_target)
                            if is_next_visible:
                                # the next target point is visible, switch to it
                                self._agents_target_index[agent_inner_index] += 1
                                # aslo update direction
                                self._agents_target_direction[agent_inner_index] = self._to_direction(target, next_target)
                                target = self._agents_targets[agent_inner_index][agent_target_index + 1]
                if self._agents_activity[agent_inner_index]:
                    if not self._continuous_moving and should_deactivate:
                        # stop calculating velocity for the agent
                        # in all other updates it will be set to zero
                        self._agents_activity[agent_inner_index] = False
                        # also clear the path
                        self._agents_path[agent_inner_index] = []
                    else:
                        # try to update the path
                        if update_path and len(self._agents_targets[agent_inner_index]) > 0:
                            target_position = self._agents_targets[agent_inner_index][-1]
                            # set the height of the start point the height of the start of the current segment
                            a_path = self.search_path((current_position[0], self._agents_height[agent_inner_index][self._agents_target_index[agent_inner_index]], current_position[1]), (target_position[0], self._agents_height[agent_inner_index][-1], target_position[1]))
                            self._set_agent_path(agent_id, a_path)  # set raw 3-float tuples path
                        if not should_deactivate:
                            to_vector = (target[0] - current_position[0], target[1] - current_position[1])
                            a_velocity = get_unit_vector(to_vector)
                            # set prefered velocity
                            rvo.set_agent_pref_velocity(sim, agent_index, (a_velocity[0] * a_speed, a_velocity[1] * a_speed))
                else:
                    rvo.set_agent_pref_velocity(sim, agent_index, (0.0, 0.0))
            else:
                rvo.set_agent_pref_velocity(sim, agent_index, (0.0, 0.0))
        # simulate in each group
        for sim in self._simulators:
            rvo.simulate(sim, delta_time, self._move_agents)
        self._last_update_time = t

    def get_all_agents_positions(self) -> List[Tuple[float, float]]:
        '''return positions, ordered by ids
        '''
        to_return: List[Tuple[float, float]] = []
        for agent_index, agent_id in enumerate(self._agents_id):
            group_index = self._agents_group[agent_index]
            sim = self._simulators[group_index]
            agent_in_group_index = self._get_agent_group_index(agent_id, self._agents_group_id[group_index])
            to_return.append(rvo.get_agent_position(sim, agent_in_group_index))
        return to_return

    def get_agent_path(self, agent_id: int) -> List[Tuple[float, float, float]]:
        agent_index = self._get_agent_inner_index(agent_id)
        return self._agents_path[agent_index]

    def get_all_agents_paths(self) -> List[List[Tuple[float, float, float]]]:
        return self._agents_path

    def get_all_agents_activities(self) -> List[bool]:
        return self._agents_activity

    def _get_agent_inner_index(self, agent_id: int) -> int:
        '''from id to total index
        '''
        for i, v in enumerate(self._agents_id):
            if v == agent_id:
                return i
        return -1

    def _get_agent_group_index(self, agent_id: int, group_ids: List[int]) -> int:
        '''return index for a given id in the array
        '''
        for i, v in enumerate(group_ids):
            if v == agent_id:
                return i
        return -1

    def get_agent_position(self, agent_id: int, agent_inner_index: Optional[int] = None) -> Tuple[float, float]:
        if agent_inner_index is None:
            agent_inner_index = self._get_agent_inner_index(agent_id)
        agent_group = self._agents_group[agent_inner_index]
        # and also we need index of the agent in the simulator
        agent_index = self._get_agent_group_index(agent_id, self._agents_group_id[agent_group])
        return rvo.get_agent_position(self._simulators[agent_group], agent_index)

    def get_agents_count(self) -> int:
        return len(self._agents_id)

    def get_agents_id(self) -> List[int]:
        '''return array of all agent ids (names) in all simulators
        '''
        return self._agents_id

    def get_active_agents_count(self) -> int:
        '''return the number of active agents
        '''
        count = 0
        for a in range(len(self._agents_id)):
            if self._agents_activity[a]:
                count += 1
        return count

    def get_default_agent_radius(self) -> float:
        return self._agent_radius

    def search_path(self, start: Tuple[float, float, float], finish: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        '''Search the path between start and finish points in the navigation mesh

        Input:
            start - 3-tuple (x, y, z) of the start point
            finish - 3-tuple (x, y, z) of the finish point

        Return:
            array in the form [(x1, y1, z1), (x2, y2, z2), ...] with coordinates of points, which form the output path
            start and finish points include as the first and the last entries in the array
            if there is no path between input points, then return empty array []
            if navmesh is not created, then return [start, finish]
        '''
        if self._navmesh is None:
            return [start, finish]
        else:
            return self._navmesh.search_path(start, finish)
