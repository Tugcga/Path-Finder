import { Heightfield, Span } from "./rc_classes";
import { RC_NULL_AREA }from "./rc_constants";
import { get_dir_offset_x, get_dir_offset_y } from "./rc_calcs";

type int = i32;
type float = f64;

export function filter_low_hanging_walkable_obstacles(walkable_climb: int,
                                                      solid: Heightfield): void{
    const w: int = solid.width;
    const h: int = solid.height;

    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let ps: Span = new Span();
            let previous_walkable: bool = false;
            let previous_area: int = RC_NULL_AREA;

            let s = solid.spans[x + y*w];
            while(s){
                let walkable: bool = s.area != RC_NULL_AREA;
                if(!walkable && previous_walkable){
                    if(Math.abs(s.smax - ps.smax) <= walkable_climb){
                        s.area = previous_area;
                    }
                }
                previous_walkable = walkable;
                previous_area = s.area;
                s = s.next;
            }
        }
    }
}


export function filter_ledge_spans(walkable_height: int,
                                   walkable_climb: int,
                                   solid: Heightfield): void{
    const w: int = solid.width;
    const h: int = solid.height;
    const MAX_HEIGHT: int = 65535;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let s = solid.spans[x + y*w];
            while(s){
                if(s.area == RC_NULL_AREA){
                    s = s.next;
                    continue;
                }
                const bot: int = s ? s.smax : 0;
                let s_next = s ? s.next : null;
                const top: int = s_next ? s_next.smin : MAX_HEIGHT;
                let minh: int = MAX_HEIGHT;

                let asmin: int = s ? s.smax : 0;
                let asmax: int = s ? s.smax : 0;

                for(let dir = 0; dir < 4; dir++){
                    const dx: int = x + get_dir_offset_x(dir);
                    const dy: int = y + get_dir_offset_y(dir);
                    if(dx < 0 || dy < 0 || dx >= w || dy >= h){
                        minh = <i32>Math.min(minh, -walkable_climb - bot);
                        continue;
                    }

                    let ns = solid.spans[dx + dy*w];
                    let nbot: int = -walkable_climb;
                    let ntop: int = ns ? ns.smin :  MAX_HEIGHT;
                    if(Math.min(top, ntop) - Math.max(bot, nbot) > walkable_height){
                        minh = <i32>Math.min(minh, nbot - bot);
                    }

                    while(ns){
                        nbot = ns.smax;
                        let ns_next = ns ? ns.next : null;
                        ntop = ns_next ? ns_next.smin : MAX_HEIGHT;
                        if(Math.min(top, ntop) - Math.max(bot, nbot) > walkable_height){
                            minh = <i32>Math.min(minh, nbot - bot);
                            if(Math.abs(nbot - bot) < walkable_climb){
                                if(nbot < asmin){
                                    asmin = nbot;
                                }
                                if(nbot > asmax){
                                    asmax = nbot;
                                }
                            }
                        }
                        ns = ns.next;
                    }
                }
                if(minh < -walkable_climb){
                    if(s){
                        s.area = RC_NULL_AREA;
                    }
                }
                else if((asmax - asmin) > walkable_climb){
                    if(s){
                        s.area = RC_NULL_AREA;
                    }
                }

                s = s ? s.next : null;
            }
        }
    }
}


export function filter_walkable_low_height_spans(walkable_height: int,
                                                 solid: Heightfield): void{
    const w: int = solid.width;
    const h: int = solid.height;
    const MAX_HEIGHT: int = 65535;
    for(let y = 0; y < h; y++){
        for(let x = 0; x < w; x++){
            let s = solid.spans[x + y*w];
            while(s){
                const bot: int = s.smax;
                let s_next = s ? s.next : null;
                const top: int = s_next ? s_next.smin : MAX_HEIGHT;
                if(top - bot < walkable_height){
                    s.area = RC_NULL_AREA;
                }
                s = s.next;
            }
        }
    }
}
