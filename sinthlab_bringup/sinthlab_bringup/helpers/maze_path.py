"""The maze path the fixture actually holds the arm to: the rails, with every corner rounded.

The rails in maze_params.yaml are zero-width segments that meet at right angles. Projected as they
are, a corner is a point where the equilibrium jumps from one rail to the next: the pull on the arm
swings through 90 degrees in one tick, felt as a snag or a kick. With `corner_radius_m` > 0 every
place where two perpendicular rails meet becomes a quarter circle of that radius, tangent to both,
so the equilibrium -- and the pull -- turns smoothly through the corner.

    corner (L):  both rails end at the point. Both are cut back by r and joined by one arc.

        ----------+            ---------.
                  |      ->               '.
                  |                         |

    fork (T):    one rail runs THROUGH the point, the other ends there. The branch is cut back by r
                 and joined to the through rail by an arc on each side; the through rail stays whole,
                 so going straight past the fork is unchanged.

    crossing (+): neither rail ends there; four arcs, nothing cut.

Rails that touch end to end in a straight line, and box-shaped corridors (the free_plane rung), are
left as they are. With corner_radius_m = 0 the path is exactly the rails.

A maze can instead be a SHAPE: a list of points joined by straight segments at any angle, usually
closed into a loop (the "0", the fish, the duck). build_shape() rounds every vertex the same way, with
an arc tangent to both segments.

Pure Python, no ROS: shared by actions/move_in_maze.py (the projection) and diagnostics/check_maze.py.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

EPS = 1e-9

Rail = Tuple[float, float, float, float]      # (a_min, a_max, b_min, b_max)


class Rect:
    """An axis-aligned rail: a segment (one axis zero-width) or a box. `rail` is its index in the YAML."""

    def __init__(self, a_min: float, a_max: float, b_min: float, b_max: float, rail: int) -> None:
        self.a_min, self.a_max, self.b_min, self.b_max, self.rail = a_min, a_max, b_min, b_max, rail

    def closest(self, qa: float, qb: float) -> Tuple[float, float, float]:
        """(a, b, squared distance) of the nearest point to (qa, qb)."""
        ca = min(max(qa, self.a_min), self.a_max)
        cb = min(max(qb, self.b_min), self.b_max)
        return ca, cb, (qa - ca) ** 2 + (qb - cb) ** 2

    def ends(self) -> List[Tuple[float, float]]:
        return [(self.a_min, self.b_min), (self.a_max, self.b_max)]


class Seg:
    """A straight segment at any angle, from p0 to p1 (a shape's side). `rail` is its index."""

    def __init__(self, p0, p1, rail: int) -> None:
        self.p0, self.p1, self.rail = tuple(p0), tuple(p1), rail
        self._d = (p1[0] - p0[0], p1[1] - p0[1])
        self._l2 = self._d[0] ** 2 + self._d[1] ** 2

    def closest(self, qa: float, qb: float) -> Tuple[float, float, float]:
        t = 0.0 if self._l2 <= EPS else ((qa - self.p0[0]) * self._d[0] + (qb - self.p0[1]) * self._d[1]) / self._l2
        t = min(max(t, 0.0), 1.0)
        pa, pb = self.p0[0] + t * self._d[0], self.p0[1] + t * self._d[1]
        return pa, pb, (qa - pa) ** 2 + (qb - pb) ** 2

    def ends(self) -> List[Tuple[float, float]]:
        return [self.p0, self.p1]


class Arc:
    """A circular arc (under 180 degrees) of radius r about (ca, cb), from `start` to `end`, tangent to
    the two pieces it joins. `rail` is the rail it leads into (the branch at a fork; the next side of a
    shape), so trial data can still name a rail."""

    def __init__(self, ca: float, cb: float, r: float, start, end, rail: int) -> None:
        self.ca, self.cb, self.r, self.start, self.end, self.rail = ca, cb, r, start, end, rail
        # The arc covers the quadrant between the two unit directions centre -> start and centre -> end.
        self._s = ((start[0] - ca) / r, (start[1] - cb) / r)
        self._e = ((end[0] - ca) / r, (end[1] - cb) / r)

    def closest(self, qa: float, qb: float) -> Tuple[float, float, float]:
        da, db = qa - self.ca, qb - self.cb
        n = math.hypot(da, db)
        if n > EPS:
            ua, ub = da / n, db / n
            # Inside the arc's wedge (always less than 180 degrees): the direction lies between the
            # start and end directions, on the same side of each as the other.
            cross = lambda x, y: x[0] * y[1] - x[1] * y[0]
            se = cross(self._s, self._e)
            if cross(self._s, (ua, ub)) * se >= -EPS and cross((ua, ub), self._e) * se >= -EPS \
                    and ua * (self._s[0] + self._e[0]) + ub * (self._s[1] + self._e[1]) >= -EPS:
                pa, pb = self.ca + self.r * ua, self.cb + self.r * ub
                return pa, pb, (qa - pa) ** 2 + (qb - pb) ** 2
        best = None
        for pa, pb in (self.start, self.end):
            d = (qa - pa) ** 2 + (qb - pb) ** 2
            if best is None or d < best[2]:
                best = (pa, pb, d)
        return best

    def ends(self) -> List[Tuple[float, float]]:
        return [self.start, self.end]


def _is_line(r: Rail) -> str:
    """'h' for a horizontal segment (b fixed), 'v' for a vertical one (a fixed), '' otherwise."""
    if abs(r[3] - r[2]) <= EPS and r[1] - r[0] > EPS:
        return "h"
    if abs(r[1] - r[0]) <= EPS and r[3] - r[2] > EPS:
        return "v"
    return ""


def build_path(rails: Sequence[Rail], corner_radius: float) -> list:
    """The elements (Rect and Arc) the projection uses. Raises ValueError if the radius does not fit."""
    rails = [tuple(float(v) for v in r) for r in rails]
    r = float(corner_radius)
    if r < 0:
        raise ValueError(f"corner_radius_m must be >= 0, got {r}")
    if r <= EPS:
        return [Rect(*rl, rail=i) for i, rl in enumerate(rails)]

    trim = {i: [0.0, 0.0] for i in range(len(rails))}       # cut back at the [low, high] end
    arcs: List[Arc] = []
    for i in range(len(rails)):
        for j in range(i + 1, len(rails)):
            ki, kj = _is_line(rails[i]), _is_line(rails[j])
            if not ki or not kj or ki == kj:
                continue                                      # only perpendicular segments round
            h, v = (rails[i], rails[j]) if ki == "h" else (rails[j], rails[i])
            hi, vi = (i, j) if ki == "h" else (j, i)
            pa, pb = v[0], h[2]                               # where they would cross
            if not (h[0] - EPS <= pa <= h[1] + EPS and v[2] - EPS <= pb <= v[3] + EPS):
                continue                                      # they do not meet
            # Directions each rail runs away from the meeting point.
            h_dirs = [d for d, ok in ((-1, pa - h[0] > EPS), (1, h[1] - pa > EPS)) if ok]
            v_dirs = [d for d, ok in ((-1, pb - v[2] > EPS), (1, v[3] - pb > EPS)) if ok]
            # A rail that ENDS here is cut back; one that runs through is left whole.
            if len(h_dirs) == 1:
                trim[hi][0 if h_dirs[0] > 0 else 1] = r
            if len(v_dirs) == 1:
                trim[vi][0 if v_dirs[0] > 0 else 1] = r
            branch = vi if len(v_dirs) == 1 else hi          # the rail that ends here
            for da in h_dirs:
                for db in v_dirs:
                    start = (pa + r * da, pb)
                    end = (pa, pb + r * db)
                    arcs.append(Arc(pa + r * da, pb + r * db, r, start, end, rail=branch))

    elements: list = []
    for i, rl in enumerate(rails):
        lo, hi_ = trim[i]
        kind = _is_line(rl)
        length = (rl[1] - rl[0]) if kind == "h" else (rl[3] - rl[2]) if kind == "v" else 0.0
        if lo + hi_ > length + EPS:
            raise ValueError(
                f"corner_radius_m {r:.3f} is too large for rail R{i + 1}: it is {length:.3f} m long and "
                f"rounded at {'both ends' if lo and hi_ else 'one end'}, so the radius may be at most "
                f"{length / (2 if lo and hi_ else 1):.3f} m")
        if kind == "h":
            elements.append(Rect(rl[0] + lo, rl[1] - hi_, rl[2], rl[3], rail=i))
        elif kind == "v":
            elements.append(Rect(rl[0], rl[1], rl[2] + lo, rl[3] - hi_, rail=i))
        else:
            elements.append(Rect(*rl, rail=i))
    return elements + arcs


def build_shape(points: Sequence[Tuple[float, float]], closed: bool, corner_radius: float) -> list:
    """A shape maze: the points joined in order (and back to the first if `closed`), every vertex
    rounded by an arc tangent to both sides. Side i runs from point i to point i+1.

    `corner_radius` is the LARGEST radius: a corner between short sides gets the largest radius that
    fits, sharing each side half-and-half with the corner at its other end. Tight corners are worth
    avoiding: close to a small arc's centre, a hand moving 1 mm sweeps the equilibrium several mm round
    the arc. Raises ValueError if the points are too few, repeated, or double back."""
    pts = [(float(a), float(b)) for a, b in points]
    if len(pts) < (3 if closed else 2):
        raise ValueError(f"a {'closed ' if closed else ''}shape needs at least {3 if closed else 2} points, got {len(pts)}")
    r = float(corner_radius)
    if r < 0:
        raise ValueError(f"corner_radius_m must be >= 0, got {r}")
    n = len(pts)
    sides = [(pts[i], pts[(i + 1) % n]) for i in range(n if closed else n - 1)]
    for i, (p0, p1) in enumerate(sides):
        if math.dist(p0, p1) <= EPS:
            raise ValueError(f"shape points {i + 1} and {(i + 1) % n + 1} are the same point")
    cut = [[0.0, 0.0] for _ in sides]           # cut back at the [start, end] of each side
    arcs = {}                                   # vertex -> its arc
    vertices = range(n) if closed else range(1, n - 1)
    for v in vertices:
        prev, nxt = (v - 1) % len(sides), v % len(sides)      # side ending at v, side starting at v
        if not closed and v == 0:
            continue
        P = pts[v]
        u1 = _unit(pts[v][0] - sides[prev][0][0], pts[v][1] - sides[prev][0][1])   # along the side into P
        u2 = _unit(sides[nxt][1][0] - P[0], sides[nxt][1][1] - P[1])               # along the side out of P
        cos_turn = u1[0] * u2[0] + u1[1] * u2[1]
        if r <= EPS or cos_turn > 1 - 1e-9:
            continue                                            # straight through: nothing to round
        if cos_turn < -1 + 1e-9:
            raise ValueError(f"shape point {v + 1} doubles back on itself")
        theta = math.acos(max(-1.0, min(1.0, -cos_turn)))      # the interior angle at P
        # Each side may give at most half its length to this corner.
        room = 0.5 * min(math.dist(*sides[prev]), math.dist(*sides[nxt]))
        if not closed:                                          # an open end has no corner to share with
            if prev == 0:
                room = min(math.dist(*sides[prev]), 0.5 * math.dist(*sides[nxt]))
            if nxt == len(sides) - 1:
                room = min(room, math.dist(*sides[nxt]))
        rv = min(r, room * math.tan(theta / 2))                 # this corner's radius
        t = rv / math.tan(theta / 2)                            # how far each side is cut back
        start = (P[0] - t * u1[0], P[1] - t * u1[1])
        end = (P[0] + t * u2[0], P[1] + t * u2[1])
        bis = _unit(u2[0] - u1[0], u2[1] - u1[1])               # points into the turn
        h = rv / math.sin(theta / 2)
        arcs[v] = Arc(P[0] + h * bis[0], P[1] + h * bis[1], rv, start, end, rail=nxt)
        cut[prev][1] = t
        cut[nxt][0] = t
    elements: list = []
    for i, (p0, p1) in enumerate(sides):
        L = math.dist(p0, p1)
        if cut[i][0] + cut[i][1] > L + 1e-9:
            raise ValueError(
                f"corner_radius_m {r:.3f} is too large for side {i + 1} (points {i + 1}->{(i + 1) % n + 1}): "
                f"it is {L:.3f} m long but the corners at its ends need {cut[i][0] + cut[i][1]:.3f} m. "
                f"Lower the radius or move the points apart.")
        u = _unit(p1[0] - p0[0], p1[1] - p0[1])
        elements.append(Seg((p0[0] + cut[i][0] * u[0], p0[1] + cut[i][0] * u[1]),
                            (p1[0] - cut[i][1] * u[0], p1[1] - cut[i][1] * u[1]), rail=i))
        if (i + 1) % n in arcs and (closed or i + 1 < n):
            elements.append(arcs[(i + 1) % n])          # in order around the shape: side, its end corner
    return elements


def _unit(a: float, b: float) -> Tuple[float, float]:
    n = math.hypot(a, b)
    return (a / n, b / n) if n > EPS else (0.0, 0.0)


def sample(elements: list, step: float = 0.01) -> List[Tuple[float, float]]:
    """Points along the whole path about `step` apart -- for plotting and reach checks."""
    out: List[Tuple[float, float]] = []
    for e in elements:
        if isinstance(e, Arc):
            a0 = math.atan2(e.start[1] - e.cb, e.start[0] - e.ca)
            a1 = math.atan2(e.end[1] - e.cb, e.end[0] - e.ca)
            sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            k = max(2, int(abs(sweep) * e.r / step) + 1)
            out += [(e.ca + e.r * math.cos(a0 + sweep * j / (k - 1)), e.cb + e.r * math.sin(a0 + sweep * j / (k - 1)))
                    for j in range(k)]
        else:
            p0, p1 = e.ends()
            k = max(2, int(math.dist(p0, p1) / step) + 1)
            out += [(p0[0] + (p1[0] - p0[0]) * j / (k - 1), p0[1] + (p1[1] - p0[1]) * j / (k - 1)) for j in range(k)]
    return out


def neighbours(elements: list) -> List[set]:
    """For each element, the elements it physically touches -- the only ones the arm can move on to."""
    def touch(x, y) -> bool:
        if isinstance(x, Rect) and isinstance(y, Rect):
            da = max(0.0, x.a_min - y.a_max, y.a_min - x.a_max)
            db = max(0.0, x.b_min - y.b_max, y.b_min - x.b_max)
            return da * da + db * db <= 1e-12
        return (any(y.closest(*p)[2] <= 1e-12 for p in x.ends())
                or any(x.closest(*p)[2] <= 1e-12 for p in y.ends()))
    return [{j for j in range(len(elements)) if j != i and touch(elements[i], elements[j])}
            for i in range(len(elements))]


def nearest(elements: list, qa: float, qb: float, candidates=None) -> Tuple[int, Tuple[float, float, float]]:
    """(index, (a, b, squared distance)) of the closest element, among `candidates` if given."""
    idx = range(len(elements)) if candidates is None else candidates
    best_i, best = None, None
    for i in idx:
        c = elements[i].closest(qa, qb)
        if best is None or c[2] < best[2]:
            best_i, best = i, c
    return best_i, best
