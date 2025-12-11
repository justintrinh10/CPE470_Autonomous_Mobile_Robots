import math
import random
import numpy as np
from typing import List, Tuple

class RRTStar:
    class Node:
        def __init__(self, x: float, y: float):
            self.x = x
            self.y = y
            self.parent = None
            self.cost = 0.0

    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        walls: List = None,
        rand_area: Tuple[float, float, float, float] = None,
        max_iter: int = 1000,
        step_size: float = 0.5,
        goal_sample_rate: float = 0.05,
        search_radius: float = 1.0,
        robot_radius: float = 0.0,
    ):
        """Initialize RRT* planner.

        - start, goal: (x, y)
        - walls: list of wall representations. Each wall may be:
            * a tuple/list (x1,y1,x2,y2)
            * an object with attributes `point1` and `point2` where each
              point has `get_x()` and `get_y()` methods (compatible with `wall.LineSegment`).
        - rand_area: (xmin, xmax, ymin, ymax). If None, computed from start/goal and walls.
        """
        self.start = self.Node(*start)
        self.goal = self.Node(*goal)
        self.walls_raw = walls or []
        self.walls = self._normalize_walls(self.walls_raw)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.robot_radius = robot_radius

        if rand_area is None:
            self.xmin, self.xmax, self.ymin, self.ymax = self._infer_bounds()
        else:
            self.xmin, self.xmax, self.ymin, self.ymax = rand_area

        self.nodes = [self.start]

    # ------------------- helpers -------------------
    def _normalize_walls(self, walls):
        normalized = []
        for w in walls:
            if w is None:
                continue
            # tuple/list style (x1,y1,x2,y2)
            if isinstance(w, (list, tuple)) and len(w) == 4:
                normalized.append(((w[0], w[1]), (w[2], w[3])))
                continue
            # object with point1 and point2 and get_x/get_y
            try:
                p1 = w.point1
                p2 = w.point2
                x1 = p1.get_x()
                y1 = p1.get_y()
                x2 = p2.get_x()
                y2 = p2.get_y()
                normalized.append(((x1, y1), (x2, y2)))
                continue
            except Exception:
                pass
            # fallback: ignore unrecognized wall
        return normalized

    def _infer_bounds(self):
        xs = [self.start.x, self.goal.x]
        ys = [self.start.y, self.goal.y]
        for (p1, p2) in self.walls:
            xs += [p1[0], p2[0]]
            ys += [p1[1], p2[1]]
        xmin = min(xs) - 1.0
        xmax = max(xs) + 1.0
        ymin = min(ys) - 1.0
        ymax = max(ys) + 1.0
        return xmin, xmax, ymin, ymax

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # ------------------- geometry utilities -------------------
    @staticmethod
    def _orientation(a, b, c):
        # returns orientation sign of triplet (a,b,c)
        return (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])

    @staticmethod
    def _on_segment(a, b, c):
        # check if point b is on segment ac
        if min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and min(a[1], c[1]) <= b[1] <= max(a[1], c[1]):
            return True
        return False

    def _segments_intersect(self, p1, p2, q1, q2):
        # robust segment intersection (no reliance on wall.LineSegment API)
        o1 = self._orientation(p1, p2, q1)
        o2 = self._orientation(p1, p2, q2)
        o3 = self._orientation(q1, q2, p1)
        o4 = self._orientation(q1, q2, p2)

        if o1 == 0 and self._on_segment(p1, q1, p2):
            return True
        if o2 == 0 and self._on_segment(p1, q2, p2):
            return True
        if o3 == 0 and self._on_segment(q1, p1, q2):
            return True
        if o4 == 0 and self._on_segment(q1, p2, q2):
            return True

        return (o1 > 0 and o2 < 0 or o1 < 0 and o2 > 0) and (o3 > 0 and o4 < 0 or o3 < 0 and o4 > 0)

    def _point_to_segment_distance(self, p, a, b):
        # shortest distance from point p to segment ab
        ax, ay = a
        bx, by = b
        px, py = p
        dx = bx - ax
        dy = by - ay
        if dx == 0 and dy == 0:
            return self._dist(p, a)
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        proj = (ax + t * dx, ay + t * dy)
        return self._dist(p, proj)

    def _segment_to_segment_distance(self, a1, a2, b1, b2):
        # minimal distance between two segments
        if self._segments_intersect(a1, a2, b1, b2):
            return 0.0
        d1 = self._point_to_segment_distance(a1, b1, b2)
        d2 = self._point_to_segment_distance(a2, b1, b2)
        d3 = self._point_to_segment_distance(b1, a1, a2)
        d4 = self._point_to_segment_distance(b2, a1, a2)
        return min(d1, d2, d3, d4)

    # ------------------- RRT* core methods -------------------
    def _sample(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        x = random.uniform(self.xmin, self.xmax)
        y = random.uniform(self.ymin, self.ymax)
        return (x, y)

    def _nearest_index(self, point):
        dlist = [self._dist((n.x, n.y), point) for n in self.nodes]
        return int(np.argmin(dlist))

    def _steer(self, from_node, to_point):
        dist = self._dist((from_node.x, from_node.y), to_point)
        if dist <= self.step_size:
            return to_point
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
        return (from_node.x + self.step_size * math.cos(theta), from_node.y + self.step_size * math.sin(theta))

    def _collision_free(self, p1, p2):
        # check collisions between the segment p1->p2 and all walls considering robot_radius
        for (w1, w2) in self.walls:
            if self._segment_to_segment_distance(p1, p2, w1, w2) <= self.robot_radius:
                return False
        return True

    def _near_indices(self, new_point):
        n = len(self.nodes) + 1
        r = self.search_radius
        dlist = [self._dist((n.x, n.y), new_point) for n in self.nodes]
        indices = [i for i, d in enumerate(dlist) if d <= r]
        return indices

    def _extract_path(self, node):
        path = [(self.goal.x, self.goal.y)]
        cur = node
        while cur is not None:
            path.append((cur.x, cur.y))
            cur = cur.parent
        path.reverse()
        return path

    def plan(self):
        """Run RRT* and return a list of (x,y) tuples representing the path.
        If no path found, returns empty list.
        """
        for i in range(self.max_iter):
            rnd = self._sample()
            nearest_idx = self._nearest_index(rnd)
            nearest_node = self.nodes[nearest_idx]
            new_point = self._steer(nearest_node, rnd)

            if not self._collision_free((nearest_node.x, nearest_node.y), new_point):
                continue

            new_node = self.Node(new_point[0], new_point[1])
            # choose parent among near nodes minimizing cost
            near_idxs = self._near_indices(new_point)
            min_cost = nearest_node.cost + self._dist((nearest_node.x, nearest_node.y), new_point)
            best_parent = nearest_node
            for idx in near_idxs:
                potential_parent = self.nodes[idx]
                if self._collision_free((potential_parent.x, potential_parent.y), new_point):
                    c = potential_parent.cost + self._dist((potential_parent.x, potential_parent.y), new_point)
                    if c < min_cost:
                        min_cost = c
                        best_parent = potential_parent

            new_node.parent = best_parent
            new_node.cost = min_cost
            self.nodes.append(new_node)

            # rewire nearby nodes
            for idx in near_idxs:
                near_node = self.nodes[idx]
                if self._collision_free((new_node.x, new_node.y), (near_node.x, near_node.y)):
                    new_cost = new_node.cost + self._dist((new_node.x, new_node.y), (near_node.x, near_node.y))
                    if new_cost < near_node.cost:
                        near_node.parent = new_node
                        near_node.cost = new_cost

            # check if we can connect to goal
            if self._dist((new_node.x, new_node.y), (self.goal.x, self.goal.y)) <= self.step_size:
                if self._collision_free((new_node.x, new_node.y), (self.goal.x, self.goal.y)):
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost + self._dist((new_node.x, new_node.y), (self.goal.x, self.goal.y))
                    return self._extract_path(self.goal)

        # if loop ends, return path to the closest node to goal if collision free
        dlist = [self._dist((n.x, n.y), (self.goal.x, self.goal.y)) for n in self.nodes]
        idx = int(np.argmin(dlist))
        closest = self.nodes[idx]
        if self._collision_free((closest.x, closest.y), (self.goal.x, self.goal.y)):
            self.goal.parent = closest
            return self._extract_path(self.goal)
        return []
