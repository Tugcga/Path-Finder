## What is it?

This repository contains Python and WASM modules, which implements path finding algorithms in navigation meshes. Both modules based on [Three Pathfinding](https://github.com/donmccurdy/three-pathfinding) library for navigation mesh and [RVO2](https://github.com/snape/RVO2) library for agents moving and collision avoiding. Versions for Python and AssemblyScript are very close to each other. If you would like to understand the algorithm, look to the Python code. It's more structured and contains more comments.

Navigation mesh path finding algorithm use three main concepts:
* structure all polygons navigation mesh to a bvh (bounding volume hierarchy)
* find closest path between centers of polygons by using A* algorithm in dual graph (vertices of this graph are centers of polygons, and two vertices are incident to one edge, iff corresponding polygons have a common edge)
* combine two previous concepts to find the path between input points

It's important to understand that the algorithm does not guaranteer the optimality of the generated path. In most cases the path is plausible.

This library does not contains methods for generating (or baking) navigation meshes. It assumes that navigation mesh is already generated. There are many tools for this:
* [Recast Navigation](https://github.com/recastnavigation/recastnavigation) This is the most famous navigation mesh solution. It allows generate mesh and find path in it
* [PyRecastDetour](https://github.com/Tugcga/PyRecastDetour) This is Python bindings for some functions in Recast Navigation. Allows to output generated navigation mesh triangulation and it polygonal description
* Blender, Unity and so on

## More detailed descriptions

For Python version is [here](python/README.md).

For AssemblyScript version is [here](wasm/README.md).