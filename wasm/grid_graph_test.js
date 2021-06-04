function getRandom(min, max) {
    return Math.random() * (max - min) + min;
}

function getRandomInt(min, max) {
    min = Math.ceil(min);
    max = Math.floor(max);
    return Math.floor(Math.random() * (max - min + 1)) + min;
}

function grid_test(exports){
	const { PerformanceObserver, performance } = require('perf_hooks');
	
	//generate graph
	let n = 500;
	let m = 500;
	let limit = 0.75;
	let vertices = new Int32Array(n * m);
	let positions = new Float32Array(n * m * 3);
	let edges = new Array();
	for(let i = 0; i < n; i++){
		for(let j = 0; j < m; j++){
			let k = m*i+j;
			vertices[k] = k;
			positions[3*k] = i;
			positions[3*k + 1] = 0.0;
			positions[3*k + 2] = j;
			
			if(Math.random() < limit){
				if(j < m - 1){
					edges.push(k);
					edges.push(k + 1);
				}
				if(i < n - 1){
					edges.push(k);
					edges.push(m*(i+1)+j);
				}
			}
		}
	}
	let edges_array = new Int32Array(edges.length);
	for(let i = 0; i < edges.length; i++){
		edges_array[i] = edges[i];
	}
	
	console.log("finish generate grid data");
	
	let positions_ptr = exports.__pin(exports.__newArray(exports.Float32Array_ID, positions));
	let vertices_ptr = exports.__pin(exports.__newArray(exports.Int32Array_ID, vertices));
	let edges_ptr = exports.__pin(exports.__newArray(exports.Int32Array_ID, edges_array));
	
	let graph_ptr = exports.__pin(exports.create_navmesh_graph(positions_ptr, vertices_ptr, edges_ptr));
	let graph = exports.NavmeshGraph.wrap(graph_ptr);
	
	console.log("finish create graph object");
	
	let steps = 20;
	for(let i = 0; i < steps; i++){
		var time = performance.now();
		let start = getRandomInt(0, n*m-1);
		let end = getRandomInt(0, n*m-1);
		let path_ptr = graph.search(start, end);
		let path = exports.__getArray(path_ptr);
		time = performance.now() - time;
		console.log(start, end, "path length:", path.length, "calc time:", time / 1000);
	}
	
	exports.__unpin(graph_ptr);
	exports.__unpin(positions_ptr);
	exports.__unpin(vertices_ptr);
	exports.__unpin(edges_ptr);
}

const fs = require("fs");
const loader = require('@assemblyscript/loader');
loader.instantiate(fs.readFileSync("./navmesh_graph.wasm"), {
    navmesh_graph: {
        "console.log"(ptr) {
            console.log(exports.__getString(ptr));
        }
    }
})
.then(module => {
    exports = module.exports;
	grid_test(exports);
})
.catch(e => {
    console.error(e);
})