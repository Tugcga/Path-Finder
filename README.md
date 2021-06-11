## What is it?

This is an implementation of algorithm for path finding in navigation mesh. The algorithm is based on [Three Pathfinding](https://github.com/donmccurdy/three-pathfinding) library. In fact, all main ideas are the same, differs only small details.  All done in Python and AssemblyScript. Path finding algorithm use three main concepts:
* structure all polygons navigation mesh to a bvh (bounding volume hierarchy)
* find closest path between centers of polygons by using A* algorithm in dual graph (vertices of this graph are centers of polygons, and two vertices are incident to one edge, iff corresponding polygons have a common edge)
* combine two previous concepts to find the path between input points

It's important to understand that the algorithm does not guaranteer the optimality of the generated path. In most cases the path is plausible.

This library does not contains methods for generating (or bakin) navigation meshes. It assumes that navigation mesh is already generated. There are many tools for this:
* [Recast Navigation](https://github.com/recastnavigation/recastnavigation) This is the most famous navigation mesh solution. It allows generate mesh and find path in it
* [PyRecastDetour](https://github.com/Tugcga/PyRecastDetour) This is Python bindings for some functions in Recast Navigation. Allows to output generated navigation mesh triangulation and it polygonal description
* Blender, Unity and so on

If you would like to understand the algorithm, look to the Python code. It's more structured and contains more comments.

## Python version

It placed on the ```python``` folder of the repo. This folder contains:

* main module ```navmesh```
* some examples
* small PySide6 application for exploring navigation meshes

### How to use

To create navigation mesh object:

```
import navmesh as nm
navmesh = nm.Navmesh(vertices, polygons)
```
* ```vertices``` is an array of the form ```[(x1, y1, z1), (x2, y2, z2), ...]``` and contains vertex coordinates
* ```polygons``` is an array of the form ```[[p11, p12, ..., p1n1], [p21, p22, p23, ..., p2n2], ...]```, where each array is a sequence of vertices, which form the polygon. Each ```pij``` is an integer - the index of the vertex in the ```vertices``` array

To find the path in the navigation mesh simply call

```
path = navmesh.serach_path(start, finish)
```

```start``` and ```finish``` are tuples with coordinates. It return the array ```path```, which contains the sequence of corners of it linear segments.

### Examples

```bvh_examples.py``` contains small benchmark for testing the speed of using bvh in the navigation mesh object. It create grid of square polygons, sample random positions and find the index of the closest polygon to each position.

```graph_examples.py``` contains examples for creating graphs and using A* algorithm for finding the shortest path in it. One of them creates grid-like graph with randomly erased edges and find path between random vertices. The result is plotted by using ```matplotlib```.

![Graph example](images/graph_01.png?raw=true)

```navmesh_examples.py``` contains some examples, which demonstrates, how to create navigation mesh from raw data and how to find the path between points. There is an example, where navigation mesh created by using raw data from the text file.

### Exploring application

This application based on [Navmesh Explorer](https://github.com/Tugcga/Navmesh-Explorer). Allows to load navigation mesh data from text file and draw path between two points. Two files ```level_triangles.txt``` and ```level_polygons.txt``` are examples of text files, which supported by application. If the navigation mesh is triangulated, then the file should contains only two rows. The first one contains plain array of vertex coordinates (divided by spaces), the second row contains plain array of triange vertex indexes. If the navigation mesh defined by polygonal description, then the file should contains three rows. The first one contains vertex coordinates, the second one - the sequence of polygon corners, and the third one - sizes of polygons.

![Application example](images/app_01.png?raw=true)

In some cases the path is not optimal. It depends on polygon decomposition. Here is an example.

![Application example](images/app_02.png?raw=true)

Start and finish points are in top and bottom polygons. At first step the algorithm finds the shortest path between these two polygons. But the mesh is symmetric, and that's why there are two equal paths - at the left side and at the right side. The algorithm select one of them (at the right side), and form result path by connecting it with input points. So, it produce non-optimal result.

## AssemblyScript version

Source files and builded WASM-module are in the ```wasm``` folder. The main purpose of this module is to use it in PlayCanvas projects. Exported functions:

* ```function create_navmesh(vertices: Float32Array, polygons: Int32Array, sizes: Int32Array): Navmesh``` Return ```Navmesh``` object. Input parameters are: ```vertices``` - plain array of vertex positions, ```polygons``` - array of polygon vertex indexes (for the first polygon, then fot the second and so on), sizes - array of polygon sizes.
* ```function create_triangle_bvh(vertices: Float32Array): TrianglesBVH``` Create bvh-structure for separate triangles. Input parameter ```vertices``` is plain array of vertex coordinates (vertices of the first triangle, then for the second one and so on).
* ```function create_bvh(vertices: Float32Array, polygons: Int32Array, sizes: Int32Array): NavmeshBVH``` Create bvh-structure for polygons. Input parameters are the same as for creating ```Navmesh```.
* ```function create_navmesh_graph(vertex_positions: Float32Array, vertices: Int32Array, edges: Int32Array): NavmeshGraph``` Create the ```NavmeshGraph``` object (actual, a graph). Input parameters are: ```vertex_positions``` - plain array of vertex coordinates, ```vertices``` - integer names of the vertices (in the same order, as it positions), ```edges``` - array with integer names of the ends of graph edges (for the first edge, then for the second and so on).

### Navmesh methods

* ```search_path(s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): Float32Array``` Find the path between ```(s_X, s_y, s_z)``` and ```(e_x, e_y, e_z)```. If there are no path between input points, then return the empty array.
* ```sample(x: f32, y: f32, z: f32): Float32Array``` Find the closest point to ```(x, y, z)``` in the navigation mesh polygons. Return the array of length 4. The first three values are coordinates of the point, and the forth value - 0.0 or 1.0. It search the closest position in some limit from initial point (0.5 by default). If there are no closest point, then the forth value in the output array is 0.0, and 1.0 if the answer is correct. It's possible to change the delta distance for search process. The function ```get_bvh_delta()``` return current value, the function ```set_bvh_delta(delta: f32)``` set the value. The new value should be set **before** navmesh creation command.
* ```get_polygon_index(x: f32, y: f32, z: f32): i32``` Return the index of the polygon, closest to the point ```(x, y, z)```. Return -1 if there are no polygons near input point.

### NavmeshBVH methods

* ```sample(x: f32, y: f32, z: f32): i32``` Return the index of the polygon, which is close to the point ```(x, y, z)```.

### TrianglesBVH methods

* ```sample(x: f32, y: f32, z: f32): Float32Array``` Find the closest point to ```(x, y, z)``` in the initial triangles.

### NavmeshGraph methods

* ```search(start_vertex: i32, end_vertex: i32): Int32Array``` Find the shortest path between graph start vertex and graph end vertex. These vertices should be set by their integer names. Return an array with sequence of vertex names, which form the path.

## Performance comparison

In this section we compare performance of three approaches: Python implementation, WASM implementation and Recast Navigation library (which is c++, but we will use Python bindings [PyRecastDetour](https://github.com/Tugcga/PyRecastDetour)). As a test scene we use the following:

![The map](images/map_00.png?raw=true)

This navigation mesh contains 2 294 polygons.

Notice, that Python and WASM implementations produce the same paths on the navigation mesh (because they use the same algorithm), but PyRecastDetour produce another path. This is an example:

This is the path from PyRecastDetour

![The path on the map](images/map_01.png?raw=true)

This is the path from out implementation

![The path on the map](images/map_02.png?raw=true)

For benchmark we generate some random pair of points and calculate the path between these points. The results in the table:

Task | Python | WASM | PyRecastDetour
--- | --- | --- | ---
Initialization time | 0.28 sec | 0.13 sec | 0.02 sec
1024 pairs | 5.05 sec | 0.35 sec | 0.08 sec
4096 pairs |  | 1.29 sec | 0.28 sec
16 384 pairs | | | 1.24 sec
