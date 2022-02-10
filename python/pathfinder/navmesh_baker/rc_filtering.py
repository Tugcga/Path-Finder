from typing import Optional
from pathfinder.navmesh_baker.rc_classes import Heightfield, Span
from pathfinder.navmesh_baker.rc_constants import RC_NULL_AREA
from pathfinder.navmesh_baker.rc_calcs import get_dir_offset_x, get_dir_offset_y

def filter_low_hanging_walkable_obstacles(walkable_climb: int,
                                          solid: Heightfield):
    w: int = solid.width
    h: int = solid.height

    for y in range(h):
        for x in range(w):
            ps: Span = Span()
            previous_walkable: bool = False
            previous_area: int = RC_NULL_AREA

            s: Optional[Span] = solid.spans[x + y*w]
            while s is not None:
                walkable: bool = s.area != RC_NULL_AREA
                # If current span is not walkable, but there is walkable
                # span just below it, mark the span above it walkable too.
                if not walkable and previous_walkable:
                    if abs(s.smax - ps.smax) <= walkable_climb:
                        s.area = previous_area
                # Copy walkable flag so that it cannot propagate
                # past multiple non-walkable objects.
                previous_walkable = walkable
                previous_area = s.area
                s = s.next


def filter_ledge_spans(walkable_height: int,
                       walkable_climb: int,
                       solid: Heightfield):
    w: int = solid.width
    h: int = solid.height
    MAX_HEIGHT: int = 65535
    for y in range(h):
        for x in range(w):
            s: Optional[Span] = solid.spans[x + y*w]
            while s is not None:
                # Skip non walkable spans
                if s.area == RC_NULL_AREA:
                    s = s.next
                    continue
                bot: int = s.smax
                top: int = s.next.smin if s.next is not None else MAX_HEIGHT
                # Find neighbours minimum height
                minh: int = MAX_HEIGHT

                # Min and max height of accessible neighbours
                asmin: int = s.smax
                asmax: int = s.smax

                for dir in range(4):
                    dx: int = x + get_dir_offset_x(dir)
                    dy: int = y + get_dir_offset_y(dir)
                    # Skip neighbours which are out of bounds
                    if dx < 0 or dy < 0 or dx >= w or dy >= h:
                        minh = min(minh, -walkable_climb - bot)
                        continue

                    # From minus infinity to the first span
                    ns: Optional[Span] = solid.spans[dx + dy*w]
                    nbot: int = -walkable_climb
                    ntop: int = ns.smin if ns is not None else MAX_HEIGHT
                    # Skip neightbour if the gap between the spans is too small
                    if min(top, ntop) - max(bot, nbot) > walkable_height:
                        minh = min(minh, nbot - bot)

                    # Rest of the spans
                    while ns is not None:
                        nbot = ns.smax
                        ntop = ns.next.smin if ns.next is not None else MAX_HEIGHT
                        # Skip neightbour if the gap between the spans is too small
                        if min(top, ntop) - max(bot, nbot) > walkable_height:
                            minh = min(minh, nbot - bot)

                            # Find min/max accessible neighbour height
                            if abs(nbot - bot) < walkable_climb:
                                if nbot < asmin:
                                    asmin = nbot
                                if nbot > asmax:
                                    asmax = nbot
                        ns = ns.next

                # The current span is close to a ledge if the drop to any
                # neighbour span is less than the walkableClimb.
                if minh < -walkable_climb:
                    s.area = RC_NULL_AREA
                elif (asmax - asmin) > walkable_climb:
                    # If the difference between all neighbours is too large,
                    # we are at steep slope, mark the span as ledge.
                    s.area = RC_NULL_AREA

                s = s.next


def filter_walkable_low_height_spans(walkable_height: int,
                                     solid: Heightfield):
    w: int = solid.width
    h: int = solid.height
    MAX_HEIGHT: int = 65535

    # Remove walkable flag from spans which do not have enough
    # space above them for the agent to stand there
    for y in range(h):
        for x in range(w):
            s: Optional[Span] = solid.spans[x + y*w]
            while s is not None:
                bot: int = s.smax
                top: int = s.next.smin if s.next is not None else MAX_HEIGHT
                if top - bot < walkable_height:
                    s.area = RC_NULL_AREA
                s = s.next
