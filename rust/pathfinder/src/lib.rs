use navmesh::{NavVec3, NavTriangle, NavResult, NavMesh, NavQuery, NavPathMode};
use js_sys::Array;

use wasm_bindgen::prelude::*;

#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[derive(Debug)]
#[wasm_bindgen]
pub struct Pathfinder {
    navmesh: NavResult<NavMesh>
}

#[wasm_bindgen]
impl Pathfinder {
    #[wasm_bindgen(constructor)]
    pub fn new(vertices: &[f32], triangles: &[u32]) -> Pathfinder {
        let vertices_count: usize = vertices.len() / 3;
        let triangles_count: usize = triangles.len() / 3;
        let mut nm_vertices: Vec<NavVec3> = Vec::with_capacity(vertices_count);
        let mut nm_triangles: Vec<NavTriangle> = Vec::with_capacity(triangles_count);

        for i in 0..vertices_count {
            nm_vertices.push(NavVec3::new(vertices[3*i], vertices[3*i + 1], vertices[3*i + 2]));
        }

        for i in 0..triangles_count {
            nm_triangles.push(NavTriangle::from([triangles[3*i], triangles[3*i + 1], triangles[3*i + 2]]));
        }

        return Pathfinder{ navmesh: NavMesh::new(nm_vertices, nm_triangles) };
    }

    #[wasm_bindgen]
    pub fn search_path(&self, s_x: f32, s_y: f32, s_z: f32, e_x: f32, e_y: f32, e_z: f32) -> Array {
        match &self.navmesh {
            Ok(nm) => {
                let path_result = nm.find_path(NavVec3::new(s_x, s_y, s_z), NavVec3::new(e_x, e_y, e_z), NavQuery::Accuracy, NavPathMode::Accuracy);
                match path_result {
                    Some(path) => {
                        let points_count = path.len();
                        let mut to_return = Vec::with_capacity(points_count * 3);
                        for i in 0..points_count {
                            let v = path[i];
                            to_return.push(v.x);
                            to_return.push(v.y);
                            to_return.push(v.z);
                        }
                        return to_return.into_iter().map(JsValue::from).collect();
                    },
                    None => {
                        return Array::new();
                    }
                }
            },
            Err(_error) => {
                return Array::new();
            }
        };
    }

    #[wasm_bindgen]
    pub fn sample(&self, x: f32, y: f32, z: f32) -> Array {
        match &self.navmesh {
            Ok(nm) => {
                let closest_result = nm.closest_point(NavVec3::new(x, y, z), NavQuery::Accuracy);
                match closest_result {
                    Some(point) => {
                        let to_return = vec![point.x, point.y, point.z];
                        return to_return.into_iter().map(JsValue::from).collect();
                    },
                    None => {
                        return Array::new();
                    }
                }
            },
            Err(_error) => {
                return Array::new();
            }
        };
    }
}