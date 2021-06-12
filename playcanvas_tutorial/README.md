## How to use Path Finder WASM module in PlayCanvas

### Step 1. Add module to the project

At first, download [the module binaries](https://github.com/Tugcga/Path-Finder/blob/main/wasm/navmesh.wasm) and add it to the project.

At second, to use WASM modules in the web, you need a special js-program - the loader. Path Finder module written on AssemplyScript, and that's why it will be better to use the loader from AssemblyScript developers. To use it, add ```https://cdn.jsdelivr.net/npm/@assemblyscript/loader/umd/index.js``` to **Settings - External Scripts**.

![Add the module](images/add_module.png?raw=true)

### Step 2. Initialize the module

We will be use the following example of the level geometry.

![Level geometry](images/map_geometry.png?raw=true)

Assume, we already generate the navigation mesh for this level. There are many external tools for this. For simplicity, we store data of the navigation mesh in text file ```level_navmesh.txt```. It contains three lines: coordinates of vertices, polygon vertex indexes and polygon sizes. But you can use any format as you like, because this data in any case will be converted to internal representation inside the module.

Ok, add this text file to the project. 

Create entity in the scene and call it, for example, ```navmesh```. Create a script ```navmesh_controller.js``` and add it to the entity. Now we a ready to coding.

We need two attributes: link to the text file with navigation mesh geometry data and link to the WASM module.

```
var NavmeshController = pc.createScript('navmeshController');
NavmeshController.attributes.add("data", {type: "asset"});
NavmeshController.attributes.add("wasm_module", {type: "asset"});
```

In the initialize method, we should load file with our navigation mesh data.

```
this.is_init = false;
var request = new XMLHttpRequest();
request.open('GET', this.data.getFileUrl(), true);
request.onload = function (msg) {
    let lines = this.response.split("\n");
    let vertices_data_array = lines[0].split(" ");
    let vertices = [];
    for(let i = 0; i < vertices_data_array.length; i++){
        vertices.push(parseFloat(vertices_data_array[i]));
    }

    let polygons = [];
    let polygons_data_array = lines[1].split(" ");
    for(let i = 0; i < polygons_data_array.length; i++){
        polygons.push(parseInt(polygons_data_array[i]));
    }

    let sizes = [];
    let sizes_data_array = lines[2].split(" ");
    for(let i = 0; i < sizes_data_array.length; i++){
        sizes.push(parseInt(sizes_data_array[i]));
    }
};
request.send();
```

Here we load the file, parse it and create three arrays: ```vertices, polygons, sizes```. Next we should load the WASM module.

```
var wasm_path = this.wasm_module.getFileUrl();
var navmesh_controller = this;
loader.instantiate(
    fetch(wasm_path),
    {
        navmesh: {
        "console.log"(ptr) {console.log(ptr);}
        }
    }).then(({exports}) => {
        navmesh_controller.init_navmesh(exports, vertices, polygons, sizes);
});
```

Here we load the module and call the method ```init_navmesh```. ```exports``` contains all functions inside the module.

```
NavmeshController.prototype.init_navmesh = function(exports, vertices, polygons, sizes){
    let vertices_pointer = exports.__pin(exports.__newArray(exports.Float32Array_ID, vertices));
    let polygons_pointer = exports.__pin(exports.__newArray(exports.Int32Array_ID, polygons));
    let sizes_pointer = exports.__pin(exports.__newArray(exports.Int32Array_ID, sizes));
    this.navmesh_pointer = exports.__pin(exports.create_navmesh(vertices_pointer, polygons_pointer, sizes_pointer));
    this.navmesh = exports.Navmesh.wrap(this.navmesh_pointer);
    this.exports = exports;
    
    exports.__unpin(vertices_pointer);
    exports.__unpin(polygons_pointer);
    exports.__unpin(sizes_pointer);
    this.is_init = true;
};
```

Here we load three arrays into the module's memory and initialize ```this.navmesh``` object. This object should be used to find paths for all entities in the scene. Then, by using ```exports.__unpin```, we allows the GC to free the memory.

### Step 3. Find the path

Inside ```navmesh_controller.js``` add the following function

```

NavmeshController.prototype.get_path = function(start, finish) {
    if(this.is_init){
        var path_ptr = this.navmesh.search_path(start.x, start.y, start.z, finish.x, finish.y, finish.z);
        var path = this.exports.__getArray(path_ptr);
        var path_points_count = path.length / 3;
        var points = [];
        for(let i = 0; i < path_points_count; i++){
            points.push(new pc.Vec3(path[3*i], path[3*i + 1], path[3*i + 2]));
        }
        return points;
    }
    else{
        return [];
    }
};
```

Here ```start``` and ```finish``` are ```pc.Vec3``` objects. We call external function ```this.navmesh.search_path``` to find the path between input positions. This function return the pointer to the array in the module's memory. So, we use ```this.exports.__getArray(path_ptr)``` to obtain actual array of floats. Next we combine each triple of floats to one ```Vec3``` position, and return the constructed array. If the module return the empty array of floats, then it mean that there are no path between input positions.

There is possibility to check, is a point inside navigation mesh or outside it. Call ```this.navmesh.get_polygon_index(point.x, point.y, point.z);``` If it return -1, then the point outside the navigation mesh.

The final code of the ```navmesh_controller.js``` is the following:

```
var NavmeshController = pc.createScript('navmeshController');
NavmeshController.attributes.add("data", {type: "asset"});
NavmeshController.attributes.add("wasm_module", {type: "asset"});

// initialize code called once per entity
NavmeshController.prototype.initialize = function() {
    this.is_init = false;
    var request = new XMLHttpRequest();
    var navmesh_controller = this;
    var app = this.app;
    var wasm_path = this.wasm_module.getFileUrl();
    request.open('GET', this.data.getFileUrl(), true);
    request.onload = function (msg) {
        let lines = this.response.split("\n");
        let vertices_data_array = lines[0].split(" ");
        let vertices = [];
        for(let i = 0; i < vertices_data_array.length; i++){
            vertices.push(parseFloat(vertices_data_array[i]));
        }

        let polygons = [];
        let polygons_data_array = lines[1].split(" ");
        for(let i = 0; i < polygons_data_array.length; i++){
            polygons.push(parseInt(polygons_data_array[i]));
        }

        let sizes = [];
        let sizes_data_array = lines[2].split(" ");
        for(let i = 0; i < sizes_data_array.length; i++){
            sizes.push(parseInt(sizes_data_array[i]));
        }
        
        loader.instantiate(
            fetch(wasm_path),
            {
                navmesh: {
                "console.log"(ptr) {console.log(ptr);}
                }
            }).then(({exports}) => {
                navmesh_controller.init_navmesh(exports, vertices, polygons, sizes);
        });
    };
    request.send();
    
    this.on("destroy", function () {
        this.exports.__unpin(this.navmesh_pointer);
    });
};

NavmeshController.prototype.get_exports = function(){
    return this.exports;
};

NavmeshController.prototype.init_navmesh = function(exports, vertices, polygons, sizes){
    let vertices_pointer = exports.__pin(exports.__newArray(exports.Float32Array_ID, vertices));
    let polygons_pointer = exports.__pin(exports.__newArray(exports.Int32Array_ID, polygons));
    let sizes_pointer = exports.__pin(exports.__newArray(exports.Int32Array_ID, sizes));
    this.navmesh_pointer = exports.__pin(exports.create_navmesh(vertices_pointer, polygons_pointer, sizes_pointer));
    this.navmesh = exports.Navmesh.wrap(this.navmesh_pointer);
    this.exports = exports;
    
    exports.__unpin(vertices_pointer);
    exports.__unpin(polygons_pointer);
    exports.__unpin(sizes_pointer);
    
    this.is_init = true;
};

NavmeshController.prototype.get_path = function(start, finish) {
    if(this.is_init){
        var path_ptr = this.navmesh.search_path(start.x, start.y, start.z, finish.x, finish.y, finish.z);
        var path = this.exports.__getArray(path_ptr);
        var path_points_count = path.length / 3;
        var points = [];
        for(let i = 0; i < path_points_count; i++){
            points.push(new pc.Vec3(path[3*i], path[3*i + 1], path[3*i + 2]));
        }
        return points;
    }
    else{
        return [];
    }
};

NavmeshController.prototype.get_point_polygon_index = function(point) {
    if(this.is_init){
        return this.navmesh.get_polygon_index(point.x, point.y, point.z);
    }
    else{
        return -1;
    }
};
```

Also, does not forget to add text file and module to the attributes inside the editor.

![Navmesh controller](images/navmesh_cotroller.png?raw=true)

### Step 4. Example application

Here is an [example](https://playcanvas.com/project/803944/overview/path-finder--example) of the small application, which use our WASM module. In this application randomly generated entities moves throw the level from one random position to the other.