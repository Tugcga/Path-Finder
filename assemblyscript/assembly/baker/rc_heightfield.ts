import { Span, Heightfield, CompactHeightfield, CompactCell, CompactSpan } from "./rc_classes";
import { RC_PI, RC_WALKABLE_AREA, RC_NULL_AREA, RC_NOT_CONNECTED } from "./rc_constants";
import { calc_tri_normal, clamp, clamp_int, get_dir_offset_x, get_dir_offset_y, set_con} from "./rc_calcs";
import { log_message } from "../common/utilities";

type int = i32;
type float = f64;

export function create_height_field(width: int,
                                    height: int,
                                    bmin: StaticArray<float>,
                                    bmax: StaticArray<float>,
                                    cs: float,
                                    ch: float): Heightfield{
    var hf = new Heightfield();
    hf.width = width;
    hf.height = height;
    hf.bmin[0] = bmin[0]; hf.bmin[1] = bmin[1]; hf.bmin[2] = bmin[2];
    hf.bmax[0] = bmax[0]; hf.bmax[1] = bmax[1]; hf.bmax[2] = bmax[2];
    hf.cs = cs;
    hf.ch = ch;
    hf.spans = new StaticArray<Span|null>(width * height);
    for(let i = 0; i < width * height; i++){
        hf.spans[i] = null;
    }

    return hf;
}

export function mark_walkable_triangles(walkable_slope_angle: float,
                                        verts: StaticArray<float>,
                                        nv: int,
                                        tris: StaticArray<int>,
                                        nt: int): StaticArray<int>{
    var areas: StaticArray<int> = new StaticArray<int>(nt);

    const walkable_thr: float = Math.cos(RC_PI * walkable_slope_angle/180.0);
    let norm: StaticArray<float> = new StaticArray<float>(3);
    var buffer_01 = new StaticArray<float>(3);
    var buffer_02 = new StaticArray<float>(3);
    var buffer_03 = new StaticArray<float>(3);
    for(let i = 0; i < nt; i++){
        const tri_0 = tris[3*i];
        const tri_1 = tris[3*i + 1];
        const tri_2 = tris[3*i + 2];
        buffer_01[0] = verts[3*tri_0]; buffer_01[1] = verts[3*tri_0 + 1]; buffer_01[2] = verts[3*tri_0 + 2];
        buffer_02[0] = verts[3*tri_1]; buffer_02[1] = verts[3*tri_1 + 1]; buffer_02[2] = verts[3*tri_1 + 2];
        buffer_03[0] = verts[3*tri_2]; buffer_03[1] = verts[3*tri_2 + 1]; buffer_03[2] = verts[3*tri_2 + 2];
        calc_tri_normal(buffer_01,
                        buffer_02, 
                        buffer_03, norm);
        if(norm[1] > walkable_thr){
            areas[i] = RC_WALKABLE_AREA;
        }
    }

    return areas;
}

function get_heightfield_span_count(hf: Heightfield): int{
    const w: int = hf.width;
    const h: int = hf.height;
    var span_count: int = 0;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let s = hf.spans[x + y*w];
            while(s){
                if(s.area != RC_NULL_AREA){
                    span_count += 1;
                }
                s = s.next;
            }
        }
    }
    return span_count;
}

export function build_compact_heightfield(walkable_height: int,
                                          walkable_climb: int,
                                          hf: Heightfield): CompactHeightfield{
    var chf = new CompactHeightfield();
    const w: int = hf.width;
    const h: int = hf.height;
    const span_count: int = get_heightfield_span_count(hf);
    chf.width = w;
    chf.height = h;
    chf.span_count = span_count;
    chf.walkable_height = walkable_height;
    chf.walkable_climb = walkable_climb;
    chf.max_regions = 0;
    chf.bmin[0] = hf.bmin[0]; chf.bmin[1] = hf.bmin[1]; chf.bmin[2] = hf.bmin[2];
    chf.bmax[0] = hf.bmax[0]; chf.bmax[1] = hf.bmax[1] + walkable_height * hf.ch; chf.bmax[2] = hf.bmax[2];
    chf.cs = hf.cs;
    chf.ch = hf.ch;
    chf.cells = new StaticArray<CompactCell|null>(w*h);
    for(let i = 0; i < w*h; i++){
        chf.cells[i] = null;
    }
    chf.spans = new StaticArray<CompactSpan|null>(span_count);
    chf.areas = new StaticArray<int>(span_count);
    for(let i = 0; i < span_count; i++){
        chf.spans[i] = null;
        chf.areas[i] = RC_NULL_AREA;
    }
    chf.dist = new StaticArray<int>(span_count);

    const MAX_HEIGHT: int = 65535;

    let idx: int = 0;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let s = hf.spans[x + y*w];
            let c: CompactCell = new CompactCell();
            chf.cells[x + y*w] = c;
            if(!s){
                continue;
            }
            c.index = idx;
            c.count = 0;
            while(s){
                if(s.area != RC_NULL_AREA){
                    const bot: int = s.smax;
                    let s_next = s ? s.next : null;
                    const top: int = s_next ? s_next.smin : MAX_HEIGHT;
                    let s_idx = chf.spans[idx];
                    if(!s_idx){
                        chf.spans[idx] = new CompactSpan();
                    }
                    let cs = chf.spans[idx];
                    if(cs){
                        cs.y = clamp_int(bot, 0, 35535);
                        cs.h = clamp_int(top - bot, 0, 255);
                    }
                    chf.areas[idx] = s.area;
                    idx += 1;
                    c.count += 1;
                }
                s = s.next;
            }
        }
    }

    const MAX_LAYERS: int = RC_NOT_CONNECTED - 1;
    let too_high_neighbour: int = 0;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let cc = chf.cells[x + y*w];
            if(cc){
                for(let i = cc.index, ilen = cc.index + cc.count; i < ilen; i++){
                    let ss = chf.spans[i];
                    if(ss){
                        for(let dir = 0; dir < 4; dir++){
                            set_con(ss, dir, RC_NOT_CONNECTED);
                            const nx: int = x + get_dir_offset_x(dir);
                            const ny: int = y + get_dir_offset_y(dir);
                            if(nx < 0 || ny < 0 || nx >= w || ny >= h){
                                continue;
                            }

                            let nc = chf.cells[nx + ny*w];
                            if(nc){
                                for(let k = nc.index, klen = nc.index + nc.count; k < klen; k++){
                                    let ns = chf.spans[k];
                                    if(ns){
                                        const bots: int = <i32>Math.max(ss.y, ns.y);
                                        const tops: int = <i32>Math.min(ss.y + ss.h, ns.y + ns.h);

                                        if(tops - bots >= walkable_height && <i32>Math.abs(ns.y - ss.y) <= walkable_climb){
                                            const lidx: int = k - nc.index;
                                            if(lidx < 0 || lidx > MAX_LAYERS){
                                                too_high_neighbour = <i32>Math.max(too_high_neighbour, lidx);
                                                continue;
                                            }
                                            set_con(ss, dir, lidx);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(too_high_neighbour > MAX_LAYERS){
        log_message("[Navmesh Baker] build_compact_heightfield: Heightfield has too many layers");
    }

    return chf;
}
