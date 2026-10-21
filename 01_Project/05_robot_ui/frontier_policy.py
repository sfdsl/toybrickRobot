"""Choose reachable observation poses inside known free space, not wall cracks."""
import heapq
import math

import cv2
import numpy as np

MIN_WALL_CLEARANCE = 0.25  # m, robot centre to occupied cells
MIN_UNKNOWN_AREA = 0.5    # m², ignore isolated unknown speckles
OBSERVATION_MAX_RANGE = 0.75


def clear_line(grid, a, b):
    steps = max(abs(a[0] - b[0]), abs(a[1] - b[1])) + 1
    xs = np.rint(np.linspace(a[0], b[0], steps)).astype(int)
    ys = np.rint(np.linspace(a[1], b[1], steps)).astype(int)
    return bool(np.all(grid[ys, xs] == 0))


def observation_goals(grid, clusters, resolution, origin, robot, excluded=()):
    """Return sorted {x,y,path,gain} poses; all paths remain in inflated free space.

    Unknown components smaller than 0.5 m² and wall-adjacent frontier cells are
    ignored. Observation poses stand back from the unknown edge; failed/visited
    poses are excluded for this exploration session.
    """
    h, w = grid.shape
    res = resolution
    obstacle_clear = cv2.distanceTransform((grid < 65).astype('uint8'), cv2.DIST_L2, 5) * res
    free_clear = cv2.distanceTransform((grid == 0).astype('uint8'), cv2.DIST_L2, 5) * res
    safe = (grid == 0) & (obstacle_clear >= MIN_WALL_CLEARANCE) & (free_clear >= 0.15)
    rx = math.floor((robot[0] - origin[0]) / res)
    ry = math.floor((robot[1] - origin[1]) / res)
    if not (0 <= rx < w and 0 <= ry < h) or grid[ry, rx] != 0:
        return []
    ys, xs = np.nonzero(safe)
    if not len(xs):
        return []
    near = np.argsort((xs - rx) ** 2 + (ys - ry) ** 2)
    start = None
    for i in near[:100]:
        if math.hypot(xs[i] - rx, ys[i] - ry) * res > 0.4:
            break
        if clear_line(grid, (rx, ry), (xs[i], ys[i])):
            start = (int(xs[i]), int(ys[i]))
            break
    if start is None:
        return []
    distances = np.full(grid.shape, np.inf)
    distances[start[1], start[0]] = 0
    queue = [(0.0, *start)]
    while queue:
        distance, x, y = heapq.heappop(queue)
        if distance > distances[y, x] + 1e-9:
            continue
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            nd = distance + res
            if 0 <= nx < w and 0 <= ny < h and safe[ny, nx] and nd < distances[ny, nx]:
                distances[ny, nx] = nd
                heapq.heappush(queue, (nd, nx, ny))
    _, labels, stats, _ = cv2.connectedComponentsWithStats((grid == -1).astype('uint8'), connectivity=4)
    minimum_unknown = max(4, math.ceil(MIN_UNKNOWN_AREA / (res * res)))
    result = []
    reach = np.isfinite(distances) & (distances >= 0.35)
    for cells, _ in clusters:
        if len(cells) * res < 0.35:
            continue
        edges = []
        for x, y in cells:
            if obstacle_clear[y, x] < MIN_WALL_CLEARANCE:
                continue
            gain = 0
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h and grid[ny, nx] == -1:
                    area = int(stats[labels[ny, nx], cv2.CC_STAT_AREA])
                    gain = max(gain, area)
            if gain >= minimum_unknown:
                edges.append((x, y, gain))
        best = None
        for x, y, gain in edges[::max(1, len(edges) // 12)]:
            radius = math.ceil(OBSERVATION_MAX_RANGE / res)
            x0, x1 = max(0, x - radius), min(w, x + radius + 1)
            y0, y1 = max(0, y - radius), min(h, y + radius + 1)
            sy, sx = np.nonzero(reach[y0:y1, x0:x1])
            candidates = []
            for tx, ty in zip(sx + x0, sy + y0):
                gap = math.hypot(tx - x, ty - y) * res
                if not 0.2 <= gap <= OBSERVATION_MAX_RANGE:
                    continue
                wx, wy = origin[0] + (tx + 0.5) * res, origin[1] + (ty + 0.5) * res
                if any(math.hypot(wx - ex, wy - ey) < 0.7 for ex, ey in excluded):
                    continue
                score = float(distances[ty, tx]) + abs(gap - 0.45) - 0.1 * min(gain * res * res, 5)
                candidates.append((score, int(tx), int(ty), wx, wy))
            for score, tx, ty, wx, wy in sorted(candidates)[:8]:
                if clear_line(grid, (tx, ty), (x, y)):
                    candidate = dict(x=wx, y=wy, path=float(distances[ty, tx]),
                                     gain=gain * res * res, score=score)
                    if best is None or score < best['score']:
                        best = candidate
                    break
        if best is not None:
            result.append(best)
    return sorted(result, key=lambda item: item['score'])
