## Rust WASM Path Finder module

This WASM module contains very limited functionality. It allows to search shortest path between two points in a navigation mesh. It based on [navmesh](https://docs.rs/navmesh/latest/navmesh/) crate for Rust. Release page contains compiled versions of the module for NodeJS ([here](https://github.com/Tugcga/Path-Finder/releases/tag/r.node.1.0)) and for the Web ([here](https://github.com/Tugcga/Path-Finder/releases/tag/r.web.1.0)).

### API

```
fn new(vertices: &[f32], triangles: &[u32]) -> Pathfinder
```

Create a new instance of the ```Pathfinder``` struct from two arrays with vertex coordinates and triangle indices. ```vertices``` is a plain array of float value. The length of this array is equal to ```x3``` of vertex count. ```triangles``` are plain array of non-negative integer values. These values define vertex indices of triangle corners.

```
fn sample(&self, x: f32, y: f32, z: f32) -> Array
```

Return an array with coordinates of the closest point in the navigation mesh with respect to point with input coordinates. If it fails to find closest point, then return an empty array


```
fn search_path(&self, s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32) -> Array
```

Return the plain array with point coordinates, which define the shortest path in the navigation mesh between start point (with coordinates ```s_x, s_y, s_z```) and finish point (with coordinates ```e_x, e_y, e_z```). If there is no path, then return an empty array.


### How to use

As example consider NodeJS version. Import the module

```
const wasm_module = require("./pathfinder.js");
```

Create the pathfinder object

```
const pathfinder = new wasm_module.Pathfinder(
	[0.0, 0.0, 0.0, // vertex 0
	 4.0, 0.0, 0.0, // vertex 1
	 4.0, 0.0, 4.0, // vertex 2
	 0.0, 0.0, 4.0], //vertex 3
	
	[0, 1, 2, // triangle 0
	 0, 2, 3]); // triangle 1
```

Calculate the shortest path between two points

```
const path = pathfinder.search_path(0.0, 0.0, 0.0, 3.0, 0.0, 3.0);
```

Find the closest point in the navigation mesh

```
const s = pathfinder.sample(2.0, 2.0, 2.0);
```


### Benchmark

We use the same benchmark as for testing the performance of the [AssemblyScript version](https://github.com/Tugcga/Path-Finder/tree/main/assemblyscript#performance-comparison). The results are not optimistic. The size of the module is 111 Kb (with respect to 31 Kb for the AssemblyScript version). The performance is very slow:

Task | Time
--- | --- 
Initialization | 0.045 sec
512 pairs | 0.61 sec
1024 pairs | 1.16 sec
2048 pairs | 2.50 sec
4096 pairs | 5.16 sec
6144 pairs | 7.75 sec

So, the module nearly x13 times slowly than AssemblyScript version. It seems that the reason on the algorithm, but not WebAssembly performance.