## AssemblyScript Path Finder module

Source files and builded WASM-module are in the ```wasm``` folder. The main purpose of this module is to use it in PlayCanvas projects. Exported functions:

* ```function create_navmesh(vertices: Float32Array, polygons: Int32Array, sizes: Int32Array): Navmesh``` Return ```Navmesh``` object. Input parameters are: ```vertices``` - plain array of vertex positions, ```polygons``` - array of polygon vertex indexes (for the first polygon, then for the second and so on), sizes - array of polygon sizes. Vertex indexes in ```polygons``` array should induce **clockwise** orientation of each polygon. It's important for search path algorithm.
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

![The map](../images/map_00.png?raw=true)

This navigation mesh contains 2 294 polygons.

Notice, that Python and WASM implementations produce the same paths on the navigation mesh (because they use the same algorithm), but PyRecastDetour produce another path. This is an example:

This is the path from PyRecastDetour

![The path on the map](../images/map_01.png?raw=true)

This is the path from out implementation

![The path on the map](../images/map_02.png?raw=true)

For benchmark we generate some random pair of points and calculate the path between these points. The results in the table:

Task | Python | WASM | PyRecastDetour
--- | --- | --- | ---
Initialization time | 0.28 sec | 0.05 sec | 0.02 sec
1024 pairs | 5.05 sec | 0.15 sec | 0.08 sec
4096 pairs |  | 0.48 sec | 0.28 sec
16 384 pairs | | 2.01 sec | 1.24 sec
38 416 pairs | | 4.62 sec | 2.69 sec
65 536 pairs | | 7.79 sec | 4.59 sec
147 456 pairs | | 17.52 sec | 10.20 sec

So, WASM version is nearly x1.7 times slowly with respect to c++ solution.