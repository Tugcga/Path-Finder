import { List } from "../common/list";
import { calc_grid_size } from "./rc_calcs";
import { create_height_field, mark_walkable_triangles, build_compact_heightfield } from "./rc_heightfield";
import { rasterize_triangles } from "./rc_rasterization";
import { filter_low_hanging_walkable_obstacles, filter_ledge_spans, filter_walkable_low_height_spans } from "./rc_filtering";
import { erode_walkable_area, build_distance_field, build_regions } from "./rc_area";
import { build_contours } from "./rc_contour";
import { ContourSet, PolyMesh } from "./rc_classes";
import { build_poly_mesh } from "./rc_mesh";

import { log_message } from "../common/utilities";

type int = i32;
type float = f64;

export class NavmeshBaker {
    m_input_vertices: List<float> = new List<float>();
    m_input_triangles: List<int>  = new List<int>();  // plain array with vertex indices for triangles

    m_output_vertices: StaticArray<float> = new StaticArray<float>(0);
    m_output_polygons: StaticArray<int> = new StaticArray<int>(0);
    m_output_sizes: StaticArray<int> = new StaticArray<int>(0);

    m_is_dirty: bool;
    constructor() {
        this.m_is_dirty = true;
    }

    add_geometry(vertices: StaticArray<f32>, polygons: StaticArray<i32>, sizes: StaticArray<i32>): void {
        const n: int = this.m_input_vertices.length / 3;
        for(let i = 0, len = vertices.length; i < len; i++){
            this.m_input_vertices.push(<f64>vertices[i]);
        }

        var shift: i32 = 0;
        for(let p_index = 0, p_count = sizes.length; p_index < p_count; p_index++){
            const p_size = unchecked(sizes[p_index]);
            for(let i = 1, len = p_size - 1; i < len; i++){
                this.m_input_triangles.push(polygons[shift] + n);
                this.m_input_triangles.push(polygons[shift + i] + n);
                this.m_input_triangles.push(polygons[shift + i + 1] + n);
            }
            shift += p_size;
        }

        this.m_is_dirty = true;
    }

    bake(cell_size: float,
         cell_height: float,
         agent_height: float,
         agent_radius: float,
         agent_max_climb: float,
         agent_max_slope: float,
         region_min_size: int,
         region_merge_size: int,
         edge_max_len: float,
         edge_max_error: float,
         verts_per_poly: int,
         detail_sample_distance: float,
         detail_sample_maximum_error: float): bool{
        const nverts = this.m_input_vertices.length / 3;
        const ntris = this.m_input_triangles.length / 3;

        var verts: StaticArray<float> = this.m_input_vertices.to_static();
        var tris: StaticArray<int> = this.m_input_triangles.to_static();

        if(nverts > 0 && ntris > 0){
            const inf = f64.MAX_VALUE;

            let min_x = inf;
            let min_y = inf;
            let min_z = inf;

            let max_x = -inf;
            let max_y = -inf;
            let max_z = -inf;

            for(let v_index = 0; v_index < nverts; v_index++){
                const x = unchecked(verts[3*v_index + 0]);
                const y = unchecked(verts[3*v_index + 1]);
                const z = unchecked(verts[3*v_index + 2]);
                if(x < min_x) min_x = x;
                if(x > max_x) max_x = x;
                if(y < min_y) min_y = y;
                if(y > max_y) max_y = y;
                if(z < min_z) min_z = z;
                if(z > max_z) max_z = z;
            }
            var bb_min: StaticArray<float> = [min_x, min_y, min_z];
            var bb_max: StaticArray<float> = [max_x, max_y, max_z];
            const cs = cell_size;
            const ch = cell_height;
            const walkable_slope_angle: float = agent_max_slope;
            const walkable_height: int = <int>Math.ceil(agent_height / ch);
            const walkable_climb: int = <int>Math.floor(agent_max_climb / ch);
            const walkable_radius: int = <int>Math.ceil(agent_radius / cs);
            const max_edge_len_float = edge_max_len / cs;
            const max_edge_len: int = <int>max_edge_len_float;
            const max_simplification_error: float = edge_max_error;
            const min_region_area: int = region_min_size*region_min_size;
            const merge_region_area: int = region_merge_size*region_merge_size;
            const max_verts_per_poly: int = verts_per_poly;

            let size_array: StaticArray<int> = calc_grid_size(bb_min, bb_max, cs);
            const width = size_array[0];
            const height = size_array[1];

            let solid = create_height_field(width, height, bb_min, bb_max, cs, ch);
            let triareas = mark_walkable_triangles(walkable_slope_angle, verts, nverts, tris, ntris);

            let is_rasterize = rasterize_triangles(verts, tris, triareas, ntris, solid, walkable_climb);
            if(!is_rasterize){
                log_message("[Navmesh Baker] bake: Could not rasterize triangles");
                return false;
            }

            filter_low_hanging_walkable_obstacles(walkable_climb, solid)
            filter_ledge_spans(walkable_height, walkable_climb, solid)
            filter_walkable_low_height_spans(walkable_height, solid)

            let chf = build_compact_heightfield(walkable_height, walkable_climb, solid);
            let is_erode: bool = erode_walkable_area(walkable_radius, chf);
            if(!is_erode){
                log_message("[Navmesh Baker] bake: Could not erode");
                return false;
            }

            let is_build_field: bool = build_distance_field(chf);
            if(!is_build_field){
                log_message("[Navmesh Baker] bake: Could not build distance field");
                return false;
            }
            let is_build_regions: bool = build_regions(chf, 0, min_region_area, merge_region_area);
            if(!is_build_regions){
                log_message("[Navmesh Baker] bake: Could not build watershed regions");
                return false;
            }
            let cset = new ContourSet();
            let is_build_contours: bool = build_contours(chf, max_simplification_error, max_edge_len, cset);
            if(!is_build_contours){
                log_message("[Navmesh Baker] bake: Could not create contours");
                return false;
            }

            let pmesh: PolyMesh = new PolyMesh();
            let is_build_polymesh: bool = build_poly_mesh(cset, max_verts_per_poly, pmesh);
            if(!is_build_polymesh){
                log_message("[Navmesh Baker] bake: Could not triangulate contours");
                return false;
            }

            let output_vertices = new StaticArray<float>(pmesh.nverts * 3);
            let pm_verts: StaticArray<i32> = pmesh.verts;
            let bmin = pmesh.bmin;
            let mesh_cs = pmesh.cs;
            let mesh_ch = pmesh.ch;
            for(let v_index = 0, v_len = pmesh.nverts; v_index < v_len; v_index++){
                let idx = 3 * v_index;
                unchecked(output_vertices[idx + 0] = unchecked(bmin[0]) + mesh_cs * pm_verts[idx + 0]);
                unchecked(output_vertices[idx + 1] = unchecked(bmin[1]) + mesh_ch * (pm_verts[idx + 1] - 1));
                unchecked(output_vertices[idx + 2] = unchecked(bmin[2]) + mesh_cs * pm_verts[idx + 2]);
            }
            this.m_output_vertices = output_vertices;
            let polygons = new List<int>();
            let polys = pmesh.polys;
            this.m_output_sizes = new StaticArray<int>(pmesh.npolys);
            for(let p_index = 0, len = pmesh.npolys; p_index < len; ++p_index){
                let pv: int = p_index * 2 * pmesh.nvp;
                let vv: int = polys[pv];
                let p_count = 0;
                while(vv != 0xffff){
                    polygons.push(vv);
                    p_count++;
                    pv += 1;
                    vv = polys[pv];
                }
                this.m_output_sizes[p_index] = p_count;
            }
            this.m_output_polygons = polygons.to_static();
            this.m_is_dirty = false;
        }
        return true;
    }

    get_navmesh_vertices(): StaticArray<float>{
        if(this.m_is_dirty){
            log_message("[Navmesh Baker] get_navmesh_vertices: Navigation mesh is not constructed. Call bake() at first");
            return new StaticArray<float>(0);
        }
        else{
            return this.m_output_vertices;
        }
    }

    get_navmesh_polygons(): StaticArray<int>{
        if(this.m_is_dirty){
            log_message("[Navmesh Baker] get_navmesh_polygons: Navigation mesh is not constructed. Call bake() at first");
            return new StaticArray<int>(0);
        }
        else{
            return this.m_output_polygons;
        }
    }

    get_navmesh_sizes(): StaticArray<int>{
        if(this.m_is_dirty){
            log_message("[Navmesh Baker] get_navmesh_sizes: Navigation mesh is not constructed. Call bake() at first");
            return new StaticArray<int>(0);
        }
        else{
            return this.m_output_sizes;
        }
    }
}
