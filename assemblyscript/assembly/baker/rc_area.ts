import { CompactHeightfield, CompactCell, CompactSpan, LevelStackEntry, DirtyEntry, Region } from "./rc_classes";
import { RC_NULL_AREA, RC_NOT_CONNECTED, RC_BORDER_REG } from "./rc_constants";
import { get_con, get_dir_offset_x, get_dir_offset_y } from "./rc_calcs";
import { List } from "../common/list";
import { log_message } from "../common/utilities";

type int = i32;
type float = f64;

export function erode_walkable_area(radius: int,
                                    chf: CompactHeightfield): bool{
    const w: int = chf.width;
    const h: int = chf.height;

    let dist: StaticArray<int> = new StaticArray<int>(chf.span_count);
    for(let i = 0; i < chf.span_count; i++){
        dist[i] = 255;
    }

    let s: CompactSpan | null = null;
    let c: CompactCell | null = null;
    let c2: CompactCell | null = null;
    let c3: CompactCell | null = null;
    let asp: CompactSpan | null = null;
    let ax: int = 0;
    let ay: int = 0;
    let ai: int = 0;
    let aax: int = 0;
    let aay: int = 0;
    let aai: int = 0;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            c = chf.cells[x + y*w];
            if(c){
                const c_index = c.index;
                const c_count = c.count;
                for(let i = c_index; i < c_index + c_count; i++){
                    if(chf.areas[i] == RC_NULL_AREA){
                        dist[i] = 0;
                    }
                    else{
                        s = chf.spans[i];
                        let nc: int = 0;
                        if(s){
                            for(let dir = 0; dir < 4; dir++){
                                if(get_con(s, dir) != RC_NOT_CONNECTED){
                                    const nx: int = x + get_dir_offset_x(dir);
                                    const ny: int = y + get_dir_offset_y(dir);
                                    c = chf.cells[nx + ny*w];
                                    if(c){
                                        const nidx: int = c.index + get_con(s, dir);
                                        if(chf.areas[nidx] != RC_NULL_AREA){
                                            nc += 1;
                                        }
                                    }
                                }
                            }
                        }
                        if(nc != 4){
                            dist[i] = 0;
                        }
                    }
                }
            }
        }
    }

    let nd: int = 0;  // 1 byte

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    s = chf.spans[i];
                    if(s){
                        if(get_con(s, 0) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(0);
                            ay = y + get_dir_offset_y(0);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 0);
                                asp = chf.spans[ai];
                                nd = <i32>Math.min(dist[ai] + 2, 255);
                                if(nd < dist[i]){
                                    dist[i] = nd;
                                }

                                if(asp){
                                    if(get_con(asp, 3) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(3);
                                        aay = ay + get_dir_offset_y(3);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 3);
                                            nd = <i32>Math.min(dist[aai] + 3, 255);
                                            if(nd < dist[i]){
                                                dist[i] = nd;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if(get_con(s, 3) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(3);
                            ay = y + get_dir_offset_y(3);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 3);
                                asp = chf.spans[ai];
                                nd = <i32>Math.min(dist[ai] + 2, 255);
                                if(nd < dist[i]){
                                    dist[i] = nd;
                                }

                                if(asp){
                                    if(get_con(asp, 2) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(2);
                                        aay = ay + get_dir_offset_y(2);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 2);
                                            nd = <i32>Math.min(dist[aai] + 3, 255);
                                            if(nd < dist[i]){
                                                dist[i] = nd;
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
    }

    for(let y = h - 1; y > -1; y--){
        for(let x = w - 1; x > -1; x--){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    s = chf.spans[i];

                    if(s){
                        if(get_con(s, 2) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(2);
                            ay = y + get_dir_offset_y(2);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 2);
                                asp = chf.spans[ai];
                                nd = <i32>Math.min(dist[ai] + 2, 255);
                                if(nd < dist[i]){
                                    dist[i] = nd;
                                }
                                if(asp){
                                    if(get_con(asp, 1) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(1);
                                        aay = ay + get_dir_offset_y(1);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 1);
                                            nd = <i32>Math.min(dist[aai] + 3, 255);
                                            if(nd < dist[i]){
                                                dist[i] = nd;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if(get_con(s, 1) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(1);
                            ay = y + get_dir_offset_y(1);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 1);
                                asp = chf.spans[ai];
                                nd = <i32>Math.min(dist[ai] + 2, 255);
                                if(nd < dist[i]){
                                    dist[i] = nd;
                                }

                                if(asp){
                                    if(get_con(asp, 0) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(0);
                                        aay = ay + get_dir_offset_y(0);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 0);
                                            nd = <i32>Math.min(dist[aai] + 3, 255);
                                            if(nd < dist[i]){
                                                dist[i] = nd;
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
    }

    const thr: int = radius * 2;  // 1 byte
    for(let i = 0; i < chf.span_count; i++){
        if(dist[i] < thr){
            chf.areas[i] = RC_NULL_AREA;
        }
    }

    return true;
}

function calculate_distance_field(chf: CompactHeightfield,
                                  src: StaticArray<int>): int{
    const w: int = chf.width;
    const h: int = chf.height;

    for(let i = 0; i < chf.span_count; i++){
        src[i] = 65535;
    }

    let c: CompactCell | null = null;
    let c2: CompactCell | null = null;
    let c3: CompactCell | null = null;
    let s: CompactSpan | null = null;
    let asp: CompactSpan | null = null;
    let ax: int = 0;
    let ay: int = 0;
    let ai: int = 0;
    let aax: int = 0;
    let aay: int = 0;
    let aai: int = 0;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    s = chf.spans[i];
                    const area: int = chf.areas[i];  // 1 byte

                    let nc: int = 0;
                    if(s){
                        for(let dir = 0; dir < 4; dir++){
                            if(get_con(s, dir) != RC_NOT_CONNECTED){
                                ax = x + get_dir_offset_x(dir);
                                ay = y + get_dir_offset_y(dir);
                                c2 = chf.cells[ax + ay*w];
                                if(c2){
                                    ai = c2.index + get_con(s, dir);
                                    if(area == chf.areas[ai]){
                                        nc += 1;
                                    }
                                }
                            }
                        }
                    }
                    if(nc != 4){
                        src[i] = 0;
                    }
                }
            }
        }
    }

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    s = chf.spans[i];

                    if(s){
                        if(get_con(s, 0) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(0);
                            ay = y + get_dir_offset_y(0);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 0);
                                asp = chf.spans[ai];
                                if(src[ai] + 2 < src[i]){
                                    src[i] = src[ai] + 2;
                                }

                                if(asp){
                                    if(get_con(asp, 3) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(3);
                                        aay = ay + get_dir_offset_y(3);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 3);
                                            if(src[aai] + 3 < src[i]){
                                                src[i] = src[aai] + 3;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if(get_con(s, 3) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(3);
                            ay = y + get_dir_offset_y(3);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 3);
                                asp = chf.spans[ai];
                                if(src[ai] + 2 < src[i]){
                                    src[i] = src[ai] + 2;
                                }

                                if(asp){
                                    if(get_con(asp, 2) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(2);
                                        aay = ay + get_dir_offset_y(2);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 2);
                                            if(src[aai] + 3 < src[i]){
                                                src[i] = src[aai] + 3;
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
    }

    for(let y = h - 1; y > -1; y--){
        for(let x = w - 1; x > -1; x--){
            c = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    s = chf.spans[i];

                    if(s){
                        if(get_con(s, 2) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(2);
                            ay = y + get_dir_offset_y(2);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 2);
                                asp = chf.spans[ai];
                                if(src[ai] + 2 < src[i]){
                                    src[i] = src[ai] + 2;
                                }

                                if(asp){
                                    if(get_con(asp, 1) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(1);
                                        aay = ay + get_dir_offset_y(1);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 1);
                                            if(src[aai] + 3 < src[i]){
                                                src[i] = src[aai] + 3;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if(get_con(s, 1) != RC_NOT_CONNECTED){
                            ax = x + get_dir_offset_x(1);
                            ay = y + get_dir_offset_y(1);
                            c2 = chf.cells[ax + ay*w];
                            if(c2){
                                ai = c2.index + get_con(s, 1);
                                asp = chf.spans[ai];
                                if(src[ai] + 2 < src[i]){
                                    src[i] = src[ai] + 2;
                                }

                                if(asp){
                                    if(get_con(asp, 0) != RC_NOT_CONNECTED){
                                        aax = ax + get_dir_offset_x(0);
                                        aay = ay + get_dir_offset_y(0);
                                        c3 = chf.cells[aax + aay*w];
                                        if(c3){
                                            aai = c3.index + get_con(asp, 0);
                                            if(src[aai] + 3 < src[i]){
                                                src[i] = src[aai] + 3;
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
    }
    let max_dist = 0;
    for(let i = 0; i < chf.span_count; i++){
        max_dist = <i32>Math.max(src[i], max_dist);
    }

    return max_dist;
}

function box_blur(chf: CompactHeightfield,
                  thr: int,
                  src: StaticArray<int>,
                  dst: StaticArray<int>): void{
    const w: int = chf.width;
    const h: int = chf.height;

    thr = thr * 2;

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let c: CompactCell | null = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    let s: CompactSpan | null = chf.spans[i];
                    const cd: int = src[i];  // 2 bytes
                    if(cd <= thr){
                        dst[i] = cd;
                        continue;
                    }

                    let d: int = cd;
                    for(let dir = 0; dir < 4; dir++){
                        if(s){
                            if(get_con(s, dir) != RC_NOT_CONNECTED){
                                const ax: int = x + get_dir_offset_x(dir);
                                const ay: int = y + get_dir_offset_y(dir);
                                let c2: CompactCell | null = chf.cells[ax + ay*w];
                                if(c2){
                                    const ai: int = c2.index + get_con(s, dir);
                                    d += src[ai];

                                    let asp: CompactSpan | null = chf.spans[ai];
                                    const dir2: int = (dir + 1) & 0x3;
                                    if(asp){
                                        if(get_con(asp, dir2) != RC_NOT_CONNECTED){
                                            const ax2: int = ax + get_dir_offset_x(dir2);
                                            const ay2: int = ay + get_dir_offset_y(dir2);
                                            let c3: CompactCell | null = chf.cells[ax2 + ay2*w];
                                            if(c3){
                                                const ai2: int = c3.index + get_con(asp, dir2);
                                                d += src[ai2];
                                            }
                                        }
                                        else{
                                            d += cd;
                                        }
                                    }
                                }
                            }
                            else{
                                d += cd * 2;
                            }
                        }
                    }
                    dst[i] = (d + 5) // 9
                }
            }
        }
    }
}

export function build_distance_field(chf: CompactHeightfield): bool{
    let src: StaticArray<int> = new StaticArray<int>(chf.span_count);  // 2 bytes per element
    let dst: StaticArray<int> = new StaticArray<int>(chf.span_count);

    chf.max_distance = calculate_distance_field(chf, src);

    box_blur(chf, 1, src, dst);
    chf.dist = dst;

    return true;
}

function paint_rect_region(minx: int,
                           maxx: int,
                           miny: int,
                           maxy: int,
                           reg_id: int,  // 2 bytes
                           chf: CompactHeightfield,
                           buf: StaticArray<int>,  // 2 bytes per element
                           src_reg: int): void{  // src_reg is a pointer to the buffer
    const w: int = chf.width;
    for(let y = miny; y < maxy; y++){
        for(let x = minx; x < maxx; x++){
            let c: CompactCell | null = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    if(chf.areas[i] != RC_NULL_AREA){
                        buf[src_reg + i] = reg_id;
                    }
                }
            }
        }
    }
}

function sort_cells_by_level(start_level: int,
                             chf: CompactHeightfield,
                             buf: StaticArray<int>,
                             src_reg: int,
                             nb_stacks: int,
                             stacks: StaticArray<List<LevelStackEntry>>,
                             log_levels_per_stack: int): void{
    const w: int = chf.width;
    const h: int = chf.height;
    start_level = start_level >> log_levels_per_stack;

    for(let j = 0; j < nb_stacks; j++){
        stacks[j].reset();
    }

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let c: CompactCell | null = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    if(chf.areas[i] == RC_NULL_AREA || buf[src_reg + i] != 0){
                        continue;
                    }

                    const level: int = chf.dist[i] >> log_levels_per_stack;
                    let s_id: int = start_level - level;
                    if(s_id >= nb_stacks){
                        continue;
                    }
                    if(s_id < 0){
                        s_id = 0;
                    }

                    stacks[s_id].push(new LevelStackEntry(x, y, i));
                }
            }
        }
    }
}

function append_stacks(src_stack: List<LevelStackEntry>,
                       dst_stack: List<LevelStackEntry>,
                       buf: StaticArray<int>,
                       src_reg: int): void{
    for(let j = 0; j < src_stack.length; j++){
        const i: int = src_stack[j].index;
        if(i < 0 || buf[src_reg + i] != 0){
            continue;
        }
        dst_stack.push(src_stack[j]);
    }
}

function expand_regions(max_iter: int,
                        level: int,
                        chf: CompactHeightfield,
                        buf: StaticArray<int>,
                        src_reg: int,
                        src_dist: int,
                        stack: List<LevelStackEntry>,
                        fill_stack: bool): void{
    const w: int = chf.width;
    const h: int = chf.height;

    let i: int = 0;
    let x: int = 0;
    let y: int = 0;
    let c: CompactCell | null = null;
    if(fill_stack){
        stack.reset();
        for(let y = 0; y < h; y++){
            for(let x = 0; x < w; x++){
                c = chf.cells[x + y*w];
                if(c){
                    for(let i = c.index; i < c.index + c.count; i++){
                        if(chf.dist[i] >= level && buf[src_reg + i] == 0 && chf.areas[i] != RC_NULL_AREA){
                            stack.push(new LevelStackEntry(x, y, i));
                        }
                    }
                }
            }
        }
    }
    else{
        for(let j = 0; j < stack.length; j++){
            i = stack[j].index;
            if(buf[src_reg + i] != 0){
                stack[j].index = -1;
            }
        }
    }

    let dirty_entries: List<DirtyEntry> = new List<DirtyEntry>();
    let iter: int = 0;
    while(stack.length > 0){
        let failed: int  = 0;
        dirty_entries.reset();

        for(let j = 0; j < stack.length; j++){
            x = stack[j].x;
            y = stack[j].y;
            i = stack[j].index;
            if(i < 0){
                failed += 1;
                continue;
            }

            let r: int = buf[src_reg + i];  // 2 bytes
            let d2: int = 0xffff;  // 65535, 2 bytes
            const area: int = chf.areas[i];  // 1 byte
            let s: CompactSpan | null = chf.spans[i];
            if(s){
                for(let dir = 0; dir < 4; dir++){
                    if(get_con(s, dir) == RC_NOT_CONNECTED){
                        continue;
                    }
                    const ax: int = x + get_dir_offset_x(dir);
                    const ay: int = y + get_dir_offset_y(dir);
                    c = chf.cells[ax + ay*w];
                    if(c){
                        const ai: int = c.index + get_con(s, dir);
                        if(chf.areas[ai] != area){
                            continue;
                        }
                        if((buf[src_reg + ai] > 0) && ((buf[src_reg + ai] & RC_BORDER_REG) == 0)){
                            if(buf[src_dist + ai] + 2 < d2){
                                r = buf[src_reg + ai];
                                d2 = buf[src_dist + ai] + 2;
                            }
                        }
                    }
                }
            }
            if(r != 0){
                stack[j].index = -1;  // mark as used
                dirty_entries.push(new DirtyEntry(i, r, d2));
            }
            else{
                failed += 1;
            }
        }
        for(let i = 0; i < dirty_entries.length; i++){
            const idx: int = dirty_entries[i].index;
            buf[src_reg + idx] = dirty_entries[i].region;
            buf[src_dist + idx] = dirty_entries[i].distance2;
        }

        if(failed == stack.length){
            break;
        }

        if(level > 0){
            iter += 1;
            if(iter >= max_iter){
                break;
            }
        }
    }
}

function flood_region(x: int,
                      y: int,
                      i: int,
                      level: int,  // 2 bytes
                      r: int,  // 2 bytes
                      chf: CompactHeightfield,
                      buf: StaticArray<int>,
                      src_reg: int,
                      src_dist: int,
                      stack: List<LevelStackEntry>): bool{
    const w: int = chf.width;
    const area: int = chf.areas[i];  // 1 byte

    stack.reset();
    stack.push(new LevelStackEntry(x, y, i));
    buf[src_reg + i] = r;
    buf[src_dist + i] = 0;

    const lev: int = level >= 2 ? level - 2 : 0;  // 1 byte
    let count: int = 0;

    let c1: CompactCell | null = null;
    let c2: CompactCell | null = null;
    let ax: int = 0;
    let ay: int = 0;
    let ai: int = 0;
    while(stack.length > 0){
        let back: LevelStackEntry = stack[stack.length - 1];
        const cx: int = back.x;
        const cy: int = back.y;
        const ci: int = back.index;
        stack.pop_last();

        let cs: CompactSpan | null = chf.spans[ci];

        let ar: int = 0;  // 1 byte
        if(cs){
            for(let dir = 0; dir < 4; dir++){
                if(get_con(cs, dir) != RC_NOT_CONNECTED){
                    ax = cx + get_dir_offset_x(dir);
                    ay = cy + get_dir_offset_y(dir);
                    c1 = chf.cells[ax + ay*w];
                    if(c1){
                        ai = c1.index + get_con(cs, dir);
                        if(chf.areas[ai] != area){
                            continue;
                        }
                        const nr: int = buf[src_reg + ai];  // 2 bytes
                        if(nr & RC_BORDER_REG){  // Do not take borders into account
                            continue;
                        }
                        if(nr != 0 && nr != r){
                            ar = nr;
                            break;
                        }

                        let asp: CompactSpan | null = chf.spans[ai];

                        const dir2: int = (dir + 1) & 0x3;
                        if(asp){
                            if(get_con(asp, dir2) != RC_NOT_CONNECTED){
                                const ax2: int = ax + get_dir_offset_x(dir2);
                                const ay2: int = ay + get_dir_offset_y(dir2);
                                c2 = chf.cells[ax2 + ay2*w];
                                if(c2){
                                    const ai2: int = c2.index + get_con(asp, dir2);
                                    if(chf.areas[ai2] != area){
                                        continue;
                                    }
                                    const nr2: int = buf[src_reg + ai2];  // 2 bytes
                                    if(nr2 != 0 && nr2 != r){
                                        ar = nr2;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(ar != 0){
                buf[src_reg + ci] = 0;
                continue;
            }

            count += 1;

            for(let dir = 0; dir < 4; dir++){
                if(get_con(cs, dir) != RC_NOT_CONNECTED){
                    ax = cx + get_dir_offset_x(dir);
                    ay = cy + get_dir_offset_y(dir);
                    c1 = chf.cells[ax + ay*w];
                    if(c1){
                        ai = c1.index + get_con(cs, dir);
                        if(chf.areas[ai] != area){
                            continue;
                        }
                        if(chf.dist[ai] >= lev && buf[src_reg + ai] == 0){
                            buf[src_reg + ai] = r;
                            buf[src_dist + ai] = 0;
                            stack.push(new LevelStackEntry(ax, ay, ai));
                        }
                    }
                }
            }
        }
    }

    return count > 0;
}

function add_unique_floor_region(reg: Region,
                                 n: int): void{
    for(let i = 0; i < reg.floors.length; i++){
        if(reg.floors[i] == n){
            return;
        }
    }
    reg.floors.push(n);
}

function is_solid_edge(chf: CompactHeightfield,
                       buf: StaticArray<int>,
                       src_reg: int,
                       x: int,
                       y: int,
                       i: int,
                       dir: int): bool{
    let s: CompactSpan | null = chf.spans[i];
    let r: int = 0;  // 2 bytes
    if(s){
        if(get_con(s, dir) != RC_NOT_CONNECTED){
            const ax: int = x + get_dir_offset_x(dir);
            const ay: int = y + get_dir_offset_y(dir);
            let c: CompactCell | null = chf.cells[ax + ay*chf.width];
            if(c){
                const ai: int = c.index + get_con(s, dir);
                r = buf[src_reg + ai];
            }
        }
    }
    if(r == buf[src_reg + i]){
        return false;
    }
    return true;
}

function walk_contour(x: int,
                     y: int,
                     i: int,
                     dir: int,
                     chf: CompactHeightfield,
                     buf: StaticArray<int>,
                     src_reg: int,
                     cont: List<int>): void{
    const start_dir: int = dir;
    const starti: int = i;

    let ss: CompactSpan | null = chf.spans[i];
    let cur_reg: int = 0;  // 2 bytes
    let ax: int = 0;
    let ay: int = 0;
    let ai: int = 0;
    let c: CompactCell | null = null;
    if(ss){
        if(get_con(ss, dir) != RC_NOT_CONNECTED){
            ax = x + get_dir_offset_x(dir);
            ay = y + get_dir_offset_y(dir);
            c = chf.cells[ax + ay*chf.width];
            if(c){
                ai = c.index + get_con(ss, dir);
                cur_reg = buf[src_reg + ai];
            }
        }
    }
    cont.push(cur_reg);

    let iter: int = 1;
    while(iter < 40000){
        let s: CompactSpan | null = chf.spans[i];
        if(s){
            if(is_solid_edge(chf, buf, src_reg, x, y, i, dir)){
                let r: int = 0;  // 2 bytes
                if(get_con(s, dir) != RC_NOT_CONNECTED){
                    ax = x + get_dir_offset_x(dir);
                    ay = y + get_dir_offset_y(dir);
                    c = chf.cells[ax + ay*chf.width];
                    if(c){
                        ai = c.index + get_con(s, dir);
                        r = buf[src_reg + ai];
                    }
                }
                if(r != cur_reg){
                    cur_reg = r;
                    cont.push(cur_reg);
                }
                dir = (dir + 1) & 0x3;  // Rotate CW
            }
            else{
                let ni: int = -1;
                const nx: int = x + get_dir_offset_x(dir);
                const ny: int = y + get_dir_offset_y(dir);
                if(get_con(s, dir) != RC_NOT_CONNECTED){
                    let nc: CompactCell | null = chf.cells[nx + ny*chf.width];
                    if(nc){
                        ni = nc.index + get_con(s, dir);
                    }
                }
                if(ni == -1){
                    return;
                }
                x = nx;
                y = ny;
                i = ni;
                dir = (dir + 3) & 0x3;  // Rotate CCW
            }
        }

        if(starti == i && start_dir == dir){
            break;
        }
        iter += 1;
    }
    if(cont.length > 1){
        let j: int = 0;
        while(j < cont.length){
            const nj: int = (j + 1) % cont.length;
            if(cont[j] == cont[nj]){
                for(let k = j; k < cont.length - 1; k++){
                    cont[k] = cont[k + 1];
                }
                cont.pop_last();
            }
            else{
                j += 1;
            }
        }
    }
}

function is_region_connected_to_border(reg: Region): bool{
    for(let i = 0; i < reg.connections.length; i++){
        if(reg.connections[i] == 0){
            return true;
        }
    }
    return false;
}

function can_merge_with_region(rega: Region, 
                               regb: Region): bool{
    if(rega.area_type != regb.area_type){
        return false;
    }
    let n: int = 0;
    for(let i = 0; i < rega.connections.length; i++){
        if(rega.connections[i] == regb.id){
            n += 1;
        }
    }
    if(n > 1){
        return false;
    }
    for(let i = 0; i < rega.floors.length; i++){
        if(rega.floors[i] == regb.id){
            return false;
        }
    }
    return true;
}

function remove_adjacent_neighbours(reg: Region): void{
    let i: int = 0;
    while(i < reg.connections.length && reg.connections.length > 1){
        const ni: int = (i+1) % reg.connections.length;
        if(reg.connections[i] == reg.connections[ni]){
            for(let j = 1; j < reg.connections.length - 1; j++){
                reg.connections[j] = reg.connections[j + 1];
            }
            reg.connections.pop_last();
        }
        else{
            i += 1;
        }
    }
}

function merge_regions(rega: Region,
                       regb: Region): bool{
    const aid: int = rega.id;  // 2 bytes
    const bid: int = regb.id;  // 2 bytes

    let acon: StaticArray<int> = new StaticArray<int>(rega.connections.length);
    for(let i = 0; i < rega.connections.length; i++){
        acon[i] = rega.connections[i];
    }
    let bcon: StaticArray<int> = regb.connections.to_static();

    let insa: int = -1;
    for(let i = 0; i < acon.length; i++){
        if(acon[i] == bid){
            insa = i;
            break;
        }
    }
    if(insa == -1){
        return false;
    }

    let insb: int = -1;
    for(let i = 0; i < bcon.length; i++){
        if(bcon[i] == aid){
            insb = i;
            break;
        }
    }
    if(insb == -1){
        return false;
    }

    rega.connections.reset();
    let ni: int = acon.length;
    for(let i = 0; i < ni - 1; i++){
        rega.connections.push(acon[(insa + 1 + i) % ni]);
    }
    ni = bcon.length;
    for(let i = 0; i < ni - 1; i++){
        rega.connections.push(bcon[(insb + 1 + i) % ni]);
    }

    remove_adjacent_neighbours(rega);

    for(let j = 0; j <regb.floors.length; j++){
        add_unique_floor_region(rega, regb.floors[j]);
    }
    rega.span_count += regb.span_count;
    regb.span_count = 0;
    regb.connections.reset();

    return true;
}

function replace_neighbour(reg: Region,
                           old_id: int,  // 2 bytes
                           new_id: int): void{  // 2 bytes
    let nei_changed: bool = false;
    for(let i = 0; i < reg.connections.length; i++){
        if(reg.connections[i] == old_id){
            reg.connections[i] = new_id;
            nei_changed = true;
        }
    }
    for(let i = 0; i < reg.floors.length; i++){
        if(reg.floors[i] == old_id){
            reg.floors[i] = new_id;
        }
    }
    if(nei_changed){
        remove_adjacent_neighbours(reg);
    }
}

function merge_and_filter_regions(min_region_area: int,
                                  merge_region_size: int,
                                  max_region_id: int,  // 2 bytes, changed in the function, so, we should also return it
                                  chf: CompactHeightfield,
                                  buf: StaticArray<int>,
                                  src_reg: int,
                                  overlaps: List<int>): StaticArray<int>{
    const w: int = chf.width;
    const h: int = chf.height;

    const nreg: int = max_region_id + 1;
    let regions: List<Region> = new List<Region>();

    for(let i = 0; i < nreg; i++){
        regions.push(new Region(i));
    }

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let c: CompactCell | null = chf.cells[x + y*w];
            if(c){
                for(let i = c.index; i < c.index + c.count; i++){
                    const r: int = buf[src_reg + i];  // 2 bytes
                    if(r == 0 || r >= nreg){
                        continue;
                    }
                    let reg: Region = regions[r];
                    reg.span_count += 1;

                    for(let j = c.index; j < c.index + c.count; j++){
                        if(i == j){
                            continue;
                        }
                        const floor_id: int = buf[src_reg + j];  // 2 bytes
                        if(floor_id == 0 || floor_id >= nreg){
                            continue;
                        }
                        if(floor_id == r){
                            reg.overlap = true;
                        }
                        add_unique_floor_region(reg, floor_id);
                    }

                    if(reg.connections.length > 0){
                        continue;
                    }

                    reg.area_type = chf.areas[i];

                    let ndir: int = -1;
                    for(let dir = 0; dir < 4; dir++){
                        if(is_solid_edge(chf, buf, src_reg, x, y, i, dir)){
                            ndir = dir;
                            break;
                        }
                    }

                    if(ndir != -1){
                        walk_contour(x, y, i, ndir, chf, buf, src_reg, reg.connections);
                    }
                }
            }
        }
    }

    let stack: List<int> = new List<int>(32);
    let trace: List<int> = new List<int>(32);
    for(let i = 0; i < nreg; i++){
        let reg1: Region = regions[i];
        if(reg1.id == 0 || (reg1.id & RC_BORDER_REG)){
            continue;
        }
        if(reg1.span_count == 0){
            continue;
        }
        if(reg1.visited){
            continue;
        }

        let connects_to_border: bool = false;
        let span_count: int = 0;
        stack.reset();
        trace.reset();

        reg1.visited = true;
        stack.push(i);

        while(stack.length > 0){
            let ri: int = stack.pop_last();
            let creg: Region = regions[ri];
            span_count += creg.span_count;
            trace.push(ri);

            for(let j = 0; j <creg.connections.length; j++){
                if(creg.connections[j] & RC_BORDER_REG){
                    connects_to_border = true;
                    continue;
                }
                let neireg: Region = regions[creg.connections[j]];
                if(neireg.visited){
                    continue;
                }
                if(neireg.id == 0 || (neireg.id & RC_BORDER_REG)){
                    continue;
                }
                stack.push(neireg.id);
                neireg.visited = true;
            }
        }
        if(span_count < min_region_area && (!connects_to_border)){
            for(let j = 0; j < trace.length; j++){
                regions[trace[j]].span_count = 0;
                regions[trace[j]].id = 0;
            }
        }
    }

    let merge_count: int = 0;
    let is_do: bool = true;
    let old_id: int = 0;
    while(is_do){
        merge_count = 0;
        for(let i = 0; i < nreg; i++){
            let reg2: Region = regions[i];
            if(reg2.id == 0 || (reg2.id & RC_BORDER_REG)){
                continue;
            }
            if(reg2.overlap){
                continue;
            }
            if(reg2.span_count == 0){
                continue;
            }

            if(reg2.span_count > merge_region_size && is_region_connected_to_border(reg2)){
                continue;
            }

            let smallest: int = 0xfffffff;  // 268 435 455
            let merge_id: int = reg2.id;  // 2 bytes
            for(let j = 0; j < reg2.connections.length; j++){
                if(reg2.connections[j] & RC_BORDER_REG){
                    continue;
                }
                let mreg: Region = regions[reg2.connections[j]];
                if(mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap){
                    continue;
                }
                if(mreg.span_count < smallest && can_merge_with_region(reg2, mreg) && can_merge_with_region(mreg, reg2)){
                    smallest = mreg.span_count;
                    merge_id = mreg.id;
                }
            }
            if(merge_id != reg2.id){
                old_id = reg2.id;  // 2 butes
                let target: Region = regions[merge_id];

                if(merge_regions(target, reg2)){
                    for(let j = 0; j < nreg; j++){
                        if(regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)){
                            continue;
                        }
                        if(regions[j].id == old_id){
                            regions[j].id = merge_id;
                        }
                        replace_neighbour(regions[j], old_id, merge_id);
                    }
                    merge_count += 1;
                }
            }
        }

        is_do = merge_count > 0;
    }

    for(let i = 0; i < nreg; i++){
        regions[i].remap = false;
        if(regions[i].id == 0){
            continue;
        }
        if(regions[i].id & RC_BORDER_REG){
            continue;
        }
        regions[i].remap = true;
    }

    let reg_id_gen: int = 0;  // 2 bytes
    for(let i = 0; i < nreg; i++){
        if(!regions[i].remap){
            continue;
        }
        old_id = regions[i].id;  // 2 bytes
        reg_id_gen += 1;
        const new_id: int = reg_id_gen;
        for(let j = i; j < nreg; j++){
            if(regions[j].id == old_id){
                regions[j].id = new_id;
                regions[j].remap = false;
            }
        }
    }
    max_region_id = reg_id_gen;

    for(let i = 0; i < chf.span_count; i++){
        if((buf[src_reg + i] & RC_BORDER_REG) == 0){
            buf[src_reg + i] = regions[buf[src_reg + i]].id;
        }
    }

    for(let i = 0; i < nreg; i++){
        if(regions[i].overlap){
            overlaps.push(regions[i].id);
        }
    }

    let to_return = new StaticArray<int>(2);
    to_return[0] = 1;
    to_return[1] = max_region_id;
    return to_return;
}

export function build_regions(chf: CompactHeightfield, 
                              border_size: int, 
                              min_region_area: int,
                              merge_region_area: int): bool{
    const w: int = chf.width;
    const h: int = chf.height;

    let buf: StaticArray<int> = new StaticArray<int>(chf.span_count * 2);  // 2 bytes per element

    const LOG_NB_STACKS: int = 3;
    const NB_STACKS: int = 1 << LOG_NB_STACKS;  // in fact = 8
    let lvl_stacks: StaticArray<List<LevelStackEntry>> = new StaticArray<List<LevelStackEntry>>(NB_STACKS);
    for(let i = 0; i < NB_STACKS; i++){
        lvl_stacks[i] = new List<LevelStackEntry>();
    }
    let stack: List<LevelStackEntry> = new List<LevelStackEntry>();

    let src_reg: int = 0;
    let src_dist: int = chf.span_count;

    let region_id: int = 1;  // 2 bytes
    let level: int = (chf.max_distance + 1) & ~1;  // equal to the first argument

    let expand_iters: int = 8;

    if(border_size > 0){
        const bw: int = <i32>Math.min(w, border_size);
        const bh: int = <i32>Math.min(h, border_size);

        paint_rect_region(0, bw, 0, h, region_id | RC_BORDER_REG, chf, buf, src_reg);
        region_id += 1;
        paint_rect_region(w - bw, w, 0, h, region_id | RC_BORDER_REG, chf, buf, src_reg);
        region_id += 1;
        paint_rect_region(0, w, 0, bh, region_id | RC_BORDER_REG, chf, buf, src_reg);
        region_id += 1;
        paint_rect_region(0, w, h - bh, h, region_id | RC_BORDER_REG, chf, buf, src_reg);
        region_id += 1;
    }

    chf.border_size = border_size;

    let s_id: int = -1;
    while(level > 0){
        level = level >= 2 ? level - 2 : 0;
        s_id = (s_id+1) & (NB_STACKS-1);

        if(s_id == 0){
            sort_cells_by_level(level, chf, buf, src_reg, NB_STACKS, lvl_stacks, 1);
        }
        else{
            append_stacks(lvl_stacks[s_id - 1], lvl_stacks[s_id], buf, src_reg);
        }

        expand_regions(expand_iters, level, chf, buf, src_reg, src_dist, lvl_stacks[s_id], false);
        for(let j = 0; j < lvl_stacks[s_id].length; j++){
            let current: LevelStackEntry = lvl_stacks[s_id][j];
            const x: int = current.x;
            const y: int = current.y;
            const i = current.index;
            if(i >= 0 && buf[src_reg + i] == 0){
                if(flood_region(x, y, i, level, region_id, chf, buf, src_reg, src_dist, stack)){
                    if(region_id == 0xFFFF){  // 65535
                        log_message("[Navmesh Baker] build_regions: Region ID overflow");
                        return false;
                    }
                    region_id += 1;
                }
            }
        }
    }

    expand_regions(expand_iters * 8, 0, chf, buf, src_reg, src_dist, stack, true);

    let overlaps: List<int> = new List<int>();
    chf.max_regions = region_id;
    let mafr_result = merge_and_filter_regions(min_region_area, merge_region_area, chf.max_regions, chf, buf, src_reg, overlaps);
    chf.max_regions = mafr_result[1];
    if(mafr_result[0] == 0){
        return false;
    }

    if(overlaps.length > 0){
        log_message("[Navmesh Baker] build_regions: " + overlaps.length.toString() + " overlapping regions");
    }

    for(let i = 0; i < chf.span_count; i++){
        let s: CompactSpan | null = chf.spans[i];
        if(s){
            s.reg = buf[src_reg + i];
        }
    }

    return true;
}
