## What is it?

This is an implementation of algorithm for path finding in navigation mesh. The algorithm is based on [Three Pathfinding](https://github.com/donmccurdy/three-pathfinding) library. In fact, all main ideas are the same, differs only small details.  All done in Python and AssemblyScript. Path finding algorithm use three main concepts:
* structure all polygons navigation mesh to a bvh (bounding volume hierarchy)
* find closest path between centers of polygons by using A* algorithm in dual graph (vertices of this graph are centers of polygons, and two vertices are incident to one edge, iff corresponding polygons have a common edge)
* combine two previous concepts to find the path between input points

Path, which return the algorithm, are not always optimal, but in most cases it is.

This library does not contains methods for generating (or bakin) navigation meshes. It assumes that navigation mesh is already generated. There are many tools for this:
* [Recast Navigation](https://github.com/recastnavigation/recastnavigation) This is the most famous navigation mesh solution. It allows generate mesh and find path in it
* [PyRecastDetour](https://github.com/Tugcga/PyRecastDetour) This is Python bindings for some functions in Recast Navigation. Allows to output generated navigation mesh triangulation and it polygonal description
* Blender, Unity and so on

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

```start``` and ```finish``` are tuples with coordinates. it return the array ```path```, which contains the sequence of corners of it linear segments.

### Examples

```bvh_examples.py``` contains small benchmark for testing the speed of using bvh in the navigation mesh object. It create grid of square polygons, sample random positions and find the index of the closest polygon to each position.

```graph_examples.py``` contains examples for creating graphs and using A* algorithm for finding the shortest path in it. One of them creates grid-like graph with randomly erased edges and find path between random vertices. The result is plotted by using ```matplotlib```.

![Graph example](images/graph_01.png?raw=true)

```navmesh_examples.py``` contains some examples, which demonstrates, how to create navigation mesh from raw data and how to find the path between points. There is an example, where navigation mesh created by using raw data from the text file.

### Exploring application

This application based on (Navmesh Explorer)[https://github.com/Tugcga/Navmesh-Explorer]. Allows to load navigation mesh data from text file and draw path between two points. Two files ```level_triangles.txt``` and ```level_polygons.txt``` are examples of text files, which supported by application. If the navigation mesh is triangulated, then the file should contains only two rows. The first one contains plain array of vertex coordinates (divided by spaces), the second row contains plain array of triange vertex indexes. If the navigation mesh defined by polygonal description, then the file should contains three rows. The first one contains vertex coordinates, the second one - the sequence of polygon corners, and the third one - sizes of polygons.

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

### Navmesh functions

* ```search_path(s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32): Float32Array``` Find the path between ```(s_X, s_y, s_z)``` and ```(e_x, e_y, e_z)```. If there are no path between input points, then return the empty array.
* ```sample(x: f32, y: f32, z: f32): Float32Array``` Find the closest point to ```(x, y, z)``` in the navigation mesh polygons. Return the array of length 4. The first three values are coordinates of the point, and the forth value - 0.0 or 1.0. It search the closest position in some limit from initial point (0.5 by default). If there are no closest point, then the forth value in the output array is 0.0, and 1.0 if the answer is correct.
* ```get_polygon_index(x: f32, y: f32, z: f32): i32``` Return the index of the polygon, closest to the point ```(x, y, z)```. Return -1 if there are no polygons near input point.
