"""Collision-checked paths for the offline simulator, not a robot controller.

Occupied/unknown cells and the map boundary are obstacles. Each allowed planning
cell is safe for the whole circular footprint, including between its centre and
edges. Segment checks traverse every crossed grid cell, including corner touches.
"""
from functools import lru_cache
import heapq
import math

ROBOT_RADIUS = .25


@lru_cache(maxsize=4)
def inflated_cells(width, height, resolution, radius, occupancy):
    blocked = bytearray(width * height)
    reach = math.ceil(radius / resolution) + 1
    rows = []
    for dy in range(-reach, reach + 1):
        # Minimum distance between the complete source and candidate cell squares.
        gap_y = max(0, abs(dy) - 1) * resolution
        if gap_y > radius + 1e-12:
            continue
        dx = math.floor(math.sqrt(max(0, radius*radius-gap_y*gap_y)) / resolution + 1 + 1e-10)
        rows.append((dy, dx))
    for index, occupied in enumerate(occupancy):
        if not occupied:
            continue
        y, x = divmod(index, width)
        for dy, dx in rows:
            yy = y + dy
            if 0 <= yy < height:
                first, last = max(0, x-dx), min(width, x+dx+1)
                blocked[yy*width+first:yy*width+last] = b'\1' * (last-first)
    edge = math.ceil(radius / resolution)
    for y in range(height):
        if y < edge or y >= height-edge:
            blocked[y*width:(y+1)*width] = b'\1' * width
        else:
            count = min(edge, width)
            blocked[y*width:y*width+count] = b'\1' * count
            blocked[(y+1)*width-count:(y+1)*width] = b'\1' * count
    return bytes(blocked)


class CollisionField:
    def __init__(self, grid, radius=ROBOT_RADIUS):
        if not math.isfinite(radius) or radius < 0:
            raise ValueError('机器人半径必须为非负有限数字')
        self.grid = grid
        self.width, self.height = grid.width, grid.height
        # Snapshot contents, not object identity: in-place obstacle edits invalidate
        # the cached inflation automatically.
        occupancy = bytes(value != 0 for value in grid.cells)
        self.blocked = inflated_cells(grid.width, grid.height, grid.resolution, radius, occupancy)

    def cell_free(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and not self.blocked[y*self.width+x]

    def point_free_cell(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return False
        ix, iy = math.floor(x), math.floor(y)
        # A point on a grid line touches both cells, and a corner touches four.
        xs = (round(x)-1, round(x)) if abs(x-round(x)) < 1e-9 else (ix,)
        ys = (round(y)-1, round(y)) if abs(y-round(y)) < 1e-9 else (iy,)
        return all(self.cell_free(cx, cy) for cx in xs for cy in ys)

    def point_free(self, point):
        return self.point_free_cell(*self.grid.world_to_cell_float(*point))

    def segment_free(self, start, goal):
        x0, y0 = self.grid.world_to_cell_float(*start)
        x1, y1 = self.grid.world_to_cell_float(*goal)
        if not self.point_free_cell(x0, y0) or not self.point_free_cell(x1, y1):
            return False
        dx, dy = x1-x0, y1-y0
        cuts = [0., 1.]
        for a, b, delta in ((x0, x1, dx), (y0, y1, dy)):
            if abs(delta) > 1e-12:
                for boundary in range(math.floor(min(a,b))+1, math.ceil(max(a,b))):
                    cuts.append((boundary-a)/delta)
        cuts = sorted(set(cuts))
        previous = 0.
        for t in cuts:
            midpoint = (previous+t)/2
            if not self.point_free_cell(x0+t*dx, y0+t*dy):
                return False
            if not self.point_free_cell(x0+midpoint*dx, y0+midpoint*dy):
                return False
            previous = t
        return True


def segment_clear(grid, start, goal, radius=ROBOT_RADIUS):
    return CollisionField(grid, radius).segment_free(start, goal)


def _astar(field, start, goal):
    grid = field.grid
    if not field.point_free(start) or not field.point_free(goal):
        return []
    if field.segment_free(start, goal):
        return [start, goal] if start != goal else [start]
    a, b = [tuple(math.floor(v) for v in grid.world_to_cell_float(*p)) for p in (start, goal)]
    root2 = math.sqrt(2)
    def heuristic(node):
        dx, dy = abs(node[0]-b[0]), abs(node[1]-b[1])
        return max(dx,dy)+(root2-1)*min(dx,dy)
    queue = [(heuristic(a), heuristic(a), 0., a)]
    costs, previous = {a: 0.}, {}
    while queue:
        _, _, cost, u = heapq.heappop(queue)
        if cost > costs[u] + 1e-12:
            continue
        if u == b:
            chain = [b]
            while chain[-1] != a:
                chain.append(previous[chain[-1]])
            centres = [grid.cell_to_world(x+.5,y+.5) for x,y in reversed(chain)]
            return [start] + centres + [goal]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            v = u[0]+dx, u[1]+dy
            if not field.cell_free(*v):
                continue
            if dx and dy and not (field.cell_free(u[0]+dx,u[1]) and field.cell_free(u[0],u[1]+dy)):
                continue
            candidate = cost + (root2 if dx and dy else 1.)
            if candidate < costs.get(v, math.inf) - 1e-12:
                costs[v], previous[v] = candidate, u
                h = heuristic(v)
                heapq.heappush(queue, (candidate+h, h, candidate, v))
    return []


def _simplify(path, field):
    result = [path[0]]
    anchor = 0
    while anchor < len(path)-1:
        end = len(path)-1
        while end > anchor+1 and not field.segment_free(path[anchor], path[end]):
            end -= 1
        if not field.segment_free(path[anchor], path[end]):
            return []
        if math.dist(result[-1], path[end]) > 1e-10:
            result.append(path[end])
        anchor = end
    return result


def _round_corners(path, field):
    result = [path[0]]
    for a,b,c in zip(path,path[1:],path[2:]):
        incoming, outgoing = math.dist(a,b), math.dist(b,c)
        trim = min(.5, incoming*.3, outgoing*.3)
        accepted = None
        for _ in range(6):
            if trim < .015:
                break
            p = tuple(b[j]+(a[j]-b[j])*trim/incoming for j in (0,1))
            q = tuple(b[j]+(c[j]-b[j])*trim/outgoing for j in (0,1))
            steps = max(4, math.ceil(2*trim/.025))
            curve = []
            for i in range(steps+1):
                t = i/steps
                curve.append(tuple((1-t)**2*p[j]+2*(1-t)*t*b[j]+t*t*q[j] for j in (0,1)))
            candidate = [result[-1]] + curve + [c]
            if all(field.segment_free(x,y) for x,y in zip(candidate,candidate[1:])):
                accepted = curve
                break
            trim *= .5
        result.extend(accepted if accepted is not None else [b])
    if len(path)>1:
        result.append(path[-1])
    return result


def _resample(path, spacing=.06):
    result = [path[0]]
    for a,b in zip(path,path[1:]):
        distance = math.dist(a,b)
        if distance < 1e-10:
            continue
        steps = max(1, math.ceil(distance/spacing))
        for i in range(1,steps+1):
            result.append(b if i==steps else tuple(a[j]+(b[j]-a[j])*i/steps for j in (0,1)))
    return result


def plan(grid, start, goal, radius=ROBOT_RADIUS):
    start, goal = tuple(start), tuple(goal)
    if not all(math.isfinite(value) for value in start+goal):
        return []
    safe = CollisionField(grid, radius)
    if not safe.point_free(start) or not safe.point_free(goal):
        return []
    # Extra clearance gives curves room at corners. A genuinely narrow passage
    # can fall back to the full footprint check without the optional 10 cm margin.
    preferred = CollisionField(grid, radius+.1)
    raw = _astar(preferred, start, goal)
    search_field = preferred
    if not raw:
        search_field = safe
        raw = _astar(safe, start, goal)
    if not raw:
        return []
    simplified = _simplify(raw, search_field)
    if not simplified:
        return []
    rounded = _round_corners(simplified, safe)
    result = _resample(rounded)
    # Validate precisely what the renderer and simulator will consume. A failed
    # rounding attempt falls back to safe straight segments, never unchecked arcs.
    if not all(safe.segment_free(a,b) for a,b in zip(result,result[1:])):
        result = _resample(simplified)
    return result if all(safe.segment_free(a,b) for a,b in zip(result,result[1:])) else []
