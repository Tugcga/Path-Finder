## Python Path Finder module

It placed on the ```python``` folder of the repo. This folder contains:

* main module ```pathfinder```
* some examples
* small PySide6 application for exploring navigation meshes

### How to use

To install:

```
pip install pynavmesh
```

To baking navigation mesh:

```
from pathfinder import navmesh_baker as nmb
# create baker object
baker = nmb.NavmeshBaker()
# add geometry, for example a simple plane
# the first array contains vertex positions, the second array contains polygons of the geometry
baker.add_geometry([(-4.0, 0.0, -4.0), (-4.0, 0.0, 4.0), (4.0, 0.0, 4.0), (4.0, 0.0, -4.0)], [[0, 1, 2, 3]])
# bake navigation mesh
baker.bake()
# obtain polygonal description of the mesh
vertices, polygons = baker.get_polygonization()
```

```baker.bake()``` can accept several parameters. The most important of them are:
* ```cell_size``` is a voxel size, used for rasterization input polygons
* ```cell_height``` is a vozel height
* ```agent_height``` suppositional agent size
* ```agent_radius``` suppositional agent radius

The following parameters define walkable areas:  ```agent_max_climb```, ```agent_max_slope```.

Other parameters define polygonization settings:
* ```region_min_size``` islands with less area will be ignored
* ```region_merge_size```
* ```edge_max_len``` maximum length of polygon edges
* ```edge_max_error```
* ```verts_per_poly``` define maximum number of corners in polygons (if this value is equal, for eaxmple, to 3, then all polygons will be triangles)
* ```detail_sample_distance```
* ```detail_sample_maximum_error```

To store generated navigation mesh to the file you can use one of two functions:
* ```baker.save_to_text(file_path)``` to save polygonal description of the navigation mesh into simple text file
* ```baker.save_to_binary(file_path)``` to save polygonal description of the navigation mesh into binary file with the same structure

To load navigation mesh data from file you can use similar functions from ```pathfinder``` module:
* ```read_from_text(file_path)```
* ```read_from_binary(file_path)```

Both functions return 2-tuple ```(vertices, polygons)```, where ```vertices``` is array of 3-tuples with vertex coordinates, ```polygons``` is array of integer arrays with polygon vertex indices.

To create path finder object:

```
import pathfinder as pf
pathfinder = pf.PathFinder(vertices, polygons)
```
* ```vertices``` is an array of the form ```[(x1, y1, z1), (x2, y2, z2), ...]``` and contains vertex coordinates
* ```polygons``` is an array of the form ```[[p11, p12, ..., p1n1], [p21, p22, p23, ..., p2n2], ...]```, where each array is a sequence of vertices, which form the polygon. Each ```pij``` is an integer - the index of the vertex in the ```vertices``` array

To find the path in the navigation mesh simply call

```
path = pathfinder.serach_path(start, finish)
```

```start``` and ```finish``` are tuples with coordinates. It return the array ```path```, which contains the sequence of corners of it linear segments.

For using agents collision avoidance it needs more detail setup. When creating ```pathfinder``` object you can setup additional parameters:
* ```neighbor_dist``` - search distance for other agents
* ```max_neighbors``` - how many agents considered fir collision avoidance
* ```time_horizon``` and ```time_horizon_obst``` - how close (in relative time) the agent should be to the other agent or collider to avoid it
* ```max_speed``` - maximum agent speed
* ```agent_radius``` - the default agent radius. This value used for building collisions (in the navigation mesh boundary) for agents
* ```update_path_find``` - how often (the interval in seconds) the system recalculate path for agents
* ```continuous_moving``` - should agents move after they achieve the target point or not
* ```move_agents``` - if ```True``` then the system move agent by internal algorithm, in other case it only calculate velocities

How the agents collision avoidance works:
* If ```vertices``` or ```polygons``` arrays are ```None``` then the system skip building navigation mesh and in this case it allows to simulate agents in infinite 2d-plane
* Implemented RVO2 algorithm assume that agent move in the plane XZ. Navigation mesh can be on different height level, but all velocities calculations use only the first (X) and third (Z) coordinates.
* It's possible to use original RVO2 compiled python module. The repository contains ```binary_rvo``` folder. Copy *.pyd file from this folder with specific python version to ```\pathfinder\pyrvo\``` folder and in ```__init__.py``` file set ```IMPORT_BINARY = True```. Binary version is faster but does not allows to delete agents from simulations.

To add agent into simulation

```
pathfinder.add_agent(position, radius, speed)
```

where ```position``` is 3-tuple of floats. This method returns the id of the new agent, his name. All agents have different unique ids.

To delete the agent from simulation

```
pathfinder.delete_agent(agent_id)
```

To set the agent destination point

```
pathfinder.set_agent_destination(agent_id, position)
```


### Examples

```baker_examples.py``` contains some basic examples, where we load some geometry into baker and generate corresponding navigation mesh.

```bvh_examples.py``` contains small benchmark for testing the speed of using bvh in the navigation mesh object. It create grid of square polygons, sample random positions and find the index of the closest polygon to each position.

```graph_examples.py``` contains examples for creating graphs and using A* algorithm for finding the shortest path in it. One of them creates grid-like graph with randomly erased edges and find path between random vertices. The result is plotted by using ```matplotlib```.

![Graph example](../images/graph_01.png?raw=true)

```navmesh_examples.py``` contains some examples, which demonstrates, how to create navigation mesh from raw data and how to find the path between points. There is an example, where navigation mesh created by using raw data from the text file.

```rvo_examples.py``` contains examples how to use raw RVO simulation and also as a sub-object of the ```PathFinder```.

### Exploring application

This application based on [Navmesh Explorer](https://github.com/Tugcga/Navmesh-Explorer). Allows to load navigation mesh data from text file, add and remove agents from this navigation mesh and move it from start to end points. Two files ```level_triangles.txt``` and ```level_polygons.txt``` are examples of text files, which supported by application. If the navigation mesh is triangulated, then the file should contains only two rows. The first one contains plain array of vertex coordinates (divided by spaces), the second row contains plain array of triangle vertex indexes. If the navigation mesh defined by polygonal description, then the file should contains three rows. The first one contains vertex coordinates, the second one - the sequence of polygon corners, and the third one - sizes of polygons.

![Application example](../images/app_01.png?raw=true)

In some cases the path is not optimal. It depends on polygon decomposition. Here is an example.

![Application example](../images/app_02.png?raw=true)

Start and finish points are in top and bottom polygons. At first step the algorithm finds the shortest path between these two polygons. But the mesh is symmetric, and that's why there are two equal paths - at the left side and at the right side. The algorithm select one of them (at the right side), and form result path by connecting it with input points. So, it produce non-optimal result.

### NavmeshBaker API

```
baker = NavmeshBaker()
```

Create a baker object.

```
baker.add_geometry(vertices: List[Tuple[float, float, float], polygons: List[List[int]])
```

Add input polygonal data to the baker. ```vertices``` is an array of 3-tuples with vertex positions, ```polygons``` is array of integer arrays with polygon indexes. If you need add several meshes to the baker, you can call this method several times with different arguments. Polygon indices should be the same as tuples indices in in ```vertices``` array.

```
baker.bake()
```

Generate navigation mesh polygonal description.

```
baker.get_polygonization()
```

Return polygonal description of the baked navigation mesh as 2-tuple ```(vertices, polygons)```.

```
baker.save_to_binary(file_path: str)
```

Save navigation mesh polygonal description into binary file.

```
baker.save_to_text(file_path: str)
```

Save navigation mesh polygonal description into text file.

### PathFinder API

```
pathfinder = PathFinder(vertices: Optional[List[Tuple[float, float, float]]] = None, 
						polygons: Optional[List[List[int]]] = None,
						neighbor_dist: float = 1.5,
						max_neighbors: int = 5,
						time_horizon: float = 1.5,
						time_horizon_obst: float = 2.0,
						max_speed: float = 10.0,
						agent_radius: float = 0.2,
						update_path_find: float = 1.0,
						continuous_moving: bool = False,
						move_agents: bool = True)
```

Create a new pathfinder object. ```vertices``` and ```polygons``` used for navigation mesh and obstacles in RVO. Other parameters used for RVO. If ```continuous_moving``` is ```True``` then all agents always try to go to the destination points. Even the are already achieve it. If ```move_agents``` is ```False``` then each ```update()``` method call does not change agents positions, but only recalculate an optimal velocities.

```
pathfinder.add_agent(position: Tuple[float, float, float], radius: float, speed: float)
```

Add new agent to the RVO simulation. Return an id of the new agent.

```
pathfinder.delete_agent(agent_id: int)
```

Remove agent from the simulation. Actual remove process start before ```update()``` method call.

```
pathfinder.set_agent_destination(agent_id, position: Tuple[float, float, float])
```

Set the target position for the agent. The pathfinder object find the shortest path from the current agent position to the destination point and start move the agent along this path.

```
pathfinder.get_all_agents_positions()
```

Return an array with positions of all agents in the simulation. Positions are 2d and contains only X and Z coordinates.

```
pathfinder.get_all_agents_paths()
```

Return an array with paths of all agents in the simulation.

```
pathfinder.get_all_agents_activities()
```

Return an array with ```True/False``` values for all agents in the simulation. ```False``` value means that corresponding agent does not move, because it comes to the final target of the path, or path is not defined.

```
pathfinder.get_agents_id()
```

Return agent ids. Use the same order as in ```get_all_agents_positions()```, ```get_all_agents_paths()``` and ```get_all_agents_activities()```.

```
pathfinder.get_agents_count()
```

Return total agent count in RVO simulation.

```
pathfinder.get_active_agents_count()
```

Return the number of active agents.

```
pathfinder.get_default_agent_radius()
```

Return default agent radius.

```
pathfinder.get_agent_position(agent_id: int)
```

Return position of an agent with a given id.


```
pathfinder.get_agent_path(agent_id: int)
```

Return move path for the given agent.

```
pathfinder.update_time()
```

Simply increase internal timer.

```
pathfinder.update()
```

Update RVO simulation. If ```move_agents = True``` then also change agent positions. The actual move shift values depends on agent speeds, calculated velocities and time between current call and previous ```update()``` or ```update_time()``` methods.

```
pathfinder.search_path(start: Tuple[float, float, float], finish: Tuple[float, float, float])
```

Return shortest path between start and finish point in the navigation mesh. If navigation mesh is not defined, then return the straight segment between start and finish positions.