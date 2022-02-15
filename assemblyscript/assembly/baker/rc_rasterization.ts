import { v_copy, v_min, v_max, overlap_bounds, clamp, clamp_int, slice } from "./rc_calcs";
import { Span, Heightfield } from "./rc_classes";
import { RC_SPAN_MAX_HEIGHT } from "./rc_constants";
import { log_message } from "../common/utilities";

type float = f64;
type int = i32;

function divide_poly(buf: StaticArray<float>,
                     in_ptr: int,
                     nin: int,
                     out1: int, out2: int,  // pointers in the buffer where we should write the data
                     x: float,
                     axis: int): StaticArray<int>{  // return nout1 and nout2
    let d: StaticArray<float> = new StaticArray<float>(12);
    for(let i = 0; i < nin; i++){
        d[i] = x - buf[3*i + axis + in_ptr];
    }
    let m: int = 0;
    let n: int = 0;

    let j: int = nin - 1;
    for(let i = 0; i < nin; i++){
        const ina = d[j] >= 0.0;
        const inb = d[i] >= 0.0;
        if(ina != inb){
            const s: float = d[j] / (d[j] - d[i]);
            buf[m*3 + out1] = buf[j*3 + in_ptr] + (buf[i*3 + in_ptr] - buf[j*3 + in_ptr]) * s;
            buf[m*3 + out1 + 1] = buf[j*3 + 1 + in_ptr] + (buf[i*3 + 1 + in_ptr] - buf[j*3 + 1 + in_ptr]) * s;
            buf[m*3 + out1 + 2] = buf[j*3 + 2 + in_ptr] + (buf[i*3 + 2 + in_ptr] - buf[j*3 + 2 + in_ptr]) * s;
            v_copy(buf, buf, out2+n*3, out1+m*3);
            m += 1;
            n += 1;
            if(d[i] > 0){
                v_copy(buf, buf, out1+m*3, in_ptr+i*3);
                m += 1;
            }
            else if(d[i] < 0){
                v_copy(buf, buf, out2+n*3, in_ptr+i*3);
                n += 1;
            }
        }
        else{
            if(d[i] >= 0){
                v_copy(buf, buf, out1+m*3, in_ptr+i*3);
                m += 1;
                if(d[i] != 0.0){
                    j = i;
                    continue;
                }
            }
            v_copy(buf, buf, out2+n*3, in_ptr+i*3);
            n += 1;
        }

        j = i;
    }

    let to_return = new StaticArray<int>(2);
    to_return[0] = m; to_return[1] = n;
    return to_return;
}

@inline
function free_span(hf: Heightfield, ptr: Span): void{
    ptr.next = hf.freelist;
    hf.freelist = ptr;
}


function add_span(hf: Heightfield,
                  x: int,
                  y: int,
                  smin: int,
                  smax: int,
                  area: int, 
                  flag_mege_thr: int): bool{
    const idx = x + y * hf.width;
    let s = new Span();
    s.smin = smin;
    s.smax = smax;
    s.area = area;
    s.next = null;

    let hf_s = hf.spans[idx];
    if(!hf_s){
        hf.spans[idx] = s;
        return true;
    }
    let prev: Span | null = null;
    let cur: Span | null = hf.spans[idx];

    while(cur){
        if(cur.smin > s.smax){
            break;
        }
        else if(cur.smax < s.smin){
            prev = cur;
            cur = cur.next;
        }
        else{
            if(cur.smin < s.smin){
                s.smin = cur.smin;
            }
            if(cur.smax > s.smax){
                s.smax = cur.smax;
            }

            if(<i32>Math.abs(s.smax - cur.smax) <= flag_mege_thr){
                s.area = <i32>Math.max(s.area, cur.area);
            }

            let next_span: Span | null = cur.next;
            free_span(hf, cur);
            if(prev){
                prev.next = next_span;
            }
            else{
                hf.spans[idx] = next_span;
            }
            cur = next_span;
        }
    }
    if(prev){
        s.next = prev.next;
        prev.next = s;
    }
    else{
        s.next = hf.spans[idx];
        hf.spans[idx] = s;
    }

    return true;
}

function rasterize_tri(v0: StaticArray<float>,
                       v1: StaticArray<float>,
                       v2: StaticArray<float>,
                       area: int,
                       hf: Heightfield,
                       bmin: StaticArray<float>,
                       bmax: StaticArray<float>,
                       cs: float,
                       ics: float,
                       ich: float,
                       flag_merge_thr: int): bool{
    const w: int = hf.width;
    const h: int = hf.height;
    let t_min = new StaticArray<float>(3);
    let t_max = new StaticArray<float>(3);
    const by: float = bmax[1] - bmin[1];

    v_copy(t_min, v0);
    v_copy(t_max, v0);
    v_min(t_min, v1);
    v_min(t_min, v2);
    v_max(t_max, v1);
    v_max(t_max, v2);

    if(!overlap_bounds(bmin, bmax, t_min, t_max)){
        return true;
    }

    let y0: int = <i32>((t_min[2] - bmin[2]) * ics);
    let y1: int = <i32>((t_max[2] - bmin[2]) * ics);
    y0 = clamp_int(y0, 0, h - 1);
    y1 = clamp_int(y1, 0, h - 1);

    let buf: StaticArray<float> = new StaticArray<float>(7 * 3 * 4);
    let in_ptr = 0;
    let inrow_ptr = 7*3;
    let p1_ptr = inrow_ptr + 7*3;
    let p2_ptr = p1_ptr + 7*3;
    v_copy(buf, v0);
    v_copy(buf, v1, 3, 0);
    v_copy(buf, v2, 6, 0);
    let nvrow = 3;
    let nv_in = 3;

    for(let y = y0; y < y1 + 1; y++){
        const cz = bmin[2] + y * cs;
        let dp_result = divide_poly(buf, in_ptr, nv_in, inrow_ptr, p1_ptr, cz+cs, 2);
        nvrow = dp_result[0];
        nv_in = dp_result[1];

        const temp = in_ptr;
        in_ptr = p1_ptr;
        p1_ptr = temp;
        if(nvrow < 3){
            continue;
        }
        let min_x = buf[inrow_ptr];
        let max_x = buf[inrow_ptr];
        for(let i = 1; i < nvrow; i++){
            if(min_x > buf[inrow_ptr + i*3]){
                min_x = buf[inrow_ptr + i*3];
            }
            if(max_x < buf[inrow_ptr + i*3]){
                max_x = buf[inrow_ptr + i*3];
            }
        }
        let x0 = <i32>((min_x - bmin[0])*ics);
        let x1 = <i32>((max_x - bmin[0])*ics);
        x0 = clamp_int(x0, 0, w - 1);
        x1 = clamp_int(x1, 0, w - 1);

        let nv: int = nvrow;
        let nv2: int = nvrow;

        for(let x = x0; x < x1 + 1; x++){
            const cx: float = bmin[0] + x*cs;
            dp_result = divide_poly(buf, inrow_ptr, nv2, p1_ptr, p2_ptr, cx+cs, 0);
            nv = dp_result[0];
            nv2 = dp_result[1];
            const temp2 = inrow_ptr;
            inrow_ptr = p2_ptr;
            p2_ptr = temp2;
            if(nv < 3){
                continue;
            }

            let smin: float = buf[p1_ptr + 1];
            let smax: float = buf[p1_ptr + 1];
            for(let i = 1; i < nv; i++){
                smin = Math.min(smin, buf[p1_ptr + i*3 + 1]);
                smax = Math.max(smax, buf[p1_ptr + i*3 + 1]);
            }
            smin -= bmin[1];
            smax -= bmin[1];
            if(smax < 0.0){
                continue;
            }
            if(smin > by){
                continue;
            }
            if(smin < 0.0){
                smin = 0.0;
            }
            if(smax > by){
                smax = by;
            }

            const ismin = clamp_int(<i32>Math.floor(smin*ich), 0, RC_SPAN_MAX_HEIGHT);
            const ismax = clamp_int(<i32>Math.ceil(smax*ich), ismin + 1, RC_SPAN_MAX_HEIGHT);

            const is_add_span = add_span(hf, x, y, ismin, ismax, area, flag_merge_thr);
            if(!is_add_span){
                return false;
            }
        }
    }

    return true;
}

export function rasterize_triangles(verts: StaticArray<float>,
                                    tris: StaticArray<int>,
                                    areas: StaticArray<int>,
                                    nt: int,
                                    solid: Heightfield,
                                    flag_merge_thr: int): bool{
    const ics: float = 1.0 / solid.cs;
    const ich: float = 1.0 / solid.ch;
    for(let i = 0; i < nt; i++){
        const i0: int = tris[3*i];
        const i1: int = tris[3*i+1];
        const i2: int = tris[3*i+2];
        let v0: StaticArray<float> = slice(verts, 3*i0, 3*i0+3);
        let v1: StaticArray<float> = slice(verts, 3*i1, 3*i1+3);
        let v2: StaticArray<float> = slice(verts, 3*i2, 3*i2+3);
        const is_rasterize: bool = rasterize_tri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flag_merge_thr);
        if(!is_rasterize){
            log_message("[Navmesh Baker] rasterize_triangles: Fails to rasterize triangles");
            return false;
        }
    }

    return true;
}
