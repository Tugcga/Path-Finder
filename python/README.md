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

To set the agent path

```
pathfinder.set_agent_path(agent_id, path)
```

where ```path``` is an array of 3-tuples. The first value of the path array is current position, so the agent will start to move to the second value in this array.


### Examples

```bvh_examples.py``` contains small benchmark for testing the speed of using bvh in the navigation mesh object. It create grid of square polygons, sample random positions and find the index of the closest polygon to each position.

```graph_examples.py``` contains examples for creating graphs and using A* algorithm for finding the shortest path in it. One of them creates grid-like graph with randomly erased edges and find path between random vertices. The result is plotted by using ```matplotlib```.

![Graph example](../images/graph_01.png?raw=true)

```navmesh_examples.py``` contains some examples, which demonstrates, how to create navigation mesh from raw data and how to find the path between points. There is an example, where navigation mesh created by using raw data from the text file.

### Exploring application

This application based on [Navmesh Explorer](https://github.com/Tugcga/Navmesh-Explorer). Allows to load navigation mesh data from text file, add and remove agents from this navigation mesh and move it from start to end points. Two files ```level_triangles.txt``` and ```level_polygons.txt``` are examples of text files, which supported by application. If the navigation mesh is triangulated, then the file should contains only two rows. The first one contains plain array of vertex coordinates (divided by spaces), the second row contains plain array of triangle vertex indexes. If the navigation mesh defined by polygonal description, then the file should contains three rows. The first one contains vertex coordinates, the second one - the sequence of polygon corners, and the third one - sizes of polygons.

![Application example](../images/app_01.png?raw=true)

In some cases the path is not optimal. It depends on polygon decomposition. Here is an example.

![Application example](../images/app_02.png?raw=true)

Start and finish points are in top and bottom polygons. At first step the algorithm finds the shortest path between these two polygons. But the mesh is symmetric, and that's why there are two equal paths - at the left side and at the right side. The algorithm select one of them (at the right side), and form result path by connecting it with input points. So, it produce non-optimal result.