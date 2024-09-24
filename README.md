## What is it?

This repository contains Python, WASM AssemblyScript and WASM Rust modules, which implements path finding algorithms in navigation meshes. Modules for Python and AssemblyScript based on [Three Pathfinding](https://github.com/donmccurdy/three-pathfinding) library for navigation mesh, [RVO2](https://github.com/snape/RVO2) library for agents moving and collision avoiding and [Recast Navigation](https://github.com/recastnavigation/recastnavigation) for navigation mesh baking. Module for Rust use [navmesh](https://docs.rs/navmesh/latest/navmesh/) crate for path finding in the navigation mesh. Versions for Python and AssemblyScript are very close to each other. If you would like to understand the algorithm, look to the Python code. It's more structured and contains more comments.

Navigation mesh path finding algorithm on non-Rust versions use three main concepts:
* structure all polygons navigation mesh to a bvh (bounding volume hierarchy)
* find closest path between centers of polygons by using A* algorithm in dual graph (vertices of this graph are centers of polygons, and two vertices are incident to one edge, iff corresponding polygons have a common edge)
* combine two previous concepts to find the path between input points

It's important to understand that the path finding algorithm does not guaranteer the optimality of the generated path. In most cases the path is plausible.

Python and AssemblyScript versions of the module allows to bake navigation mesh from input geometry. The algorithm is in fact a port of one specific method from [Recast Navigation](https://github.com/recastnavigation/recastnavigation). For baking purpose you can also use other solutions:
* Original [Recast Navigation](https://github.com/recastnavigation/recastnavigation). This is the most famous navigation mesh solution. It allows generate mesh and find path in it
* [PyRecastDetour](https://github.com/Tugcga/PyRecastDetour) This is Python bindings for some functions in Recast Navigation. Allows to output generated navigation mesh triangulation and it polygonal description
* Blender, Unity and so on

## More detailed descriptions

For Python version: [here](python/).

For AssemblyScript version: [here](assemblyscript/).

For Rust version: [here](rust/pathfinder/).

## License

Copyright (c) 2021-2024 Shekn Itrch

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 

## Citation

```
Title: Path Finder
Version: C.1
DOI: 10.5281/zenodo.13832416 
Release Data: 2024-09-24
URL: https://github.com/Tugcga/Path-Finder
```