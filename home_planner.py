"""Collision-free joint-space path search to a home pose.

Extracted from lerobot_robot_mantis/mantis_follower.py so the teleop's HOME button and the
policy runner can offer the same routes. It owns no robot state and imports nothing from
either stack: the caller passes a `margin_at(q_arm, floor_scale) -> (margin, pair_name)`
callable and the joint limits, so the only contract is that margin >= 0 means "clear".

The straight segment is tier one, so whenever the plain ramp is legal this returns exactly
that and the search never runs.
"""

import time

import numpy as np


class HomePlanner:
    """Search for a collision-free joint path q0 -> home.

    Tiers run cheapest and most predictable first, so an arm that only needs to fold its
    wrist out of the way does not get an RRT's random detour:
      direct  - the straight segment, i.e. what the HOME button does today;
      staged  - one joint GROUP at a time (wrists home first, then the arm, ...);
      greedy  - one JOINT at a time, always taking the branch with the best clearance;
      detour  - back one joint out by 45/90/135 deg first, then re-try the above;
      rrt     - RRT-Connect over the arm joints, shortcut-smoothed.
    Every segment of every returned path is sampled through the same collision floors the
    barrier enforces, at the caller's own check_step, so an accepted plan is certified
    motion rather than an approximation of it.

    FLOOR RELAXATION. Those floors are a safety cushion (d_min 15 mm, table 25 mm) on top
    of real contact, and they are what a blocked home is usually blocked by. When nothing
    routes at the full cushion the whole search repeats at a fraction of it, and finally at
    0 - real geometric contact only. An arm parked in a corner is not safer than one that
    walks out with 8 mm of clearance instead of 25.

    NON-WORSENING. Whatever the scale, no pose on the path may be worse than the pose the
    arm is ALREADY in (floor_min below). That is what lets a start which is itself inside
    the cushion - exactly the case a straight-path check refuses, because s=0 already fails
    - be left at all, and lets it be left only in a direction that does not dig deeper.
    """

    def __init__(self, margin_at, arm_names, lo, hi, check_step,
                 floor_scales=(1.0, 0.5, 0.0), rrt_step=0.35, smooth_time_s=2.0,
                 allow_partial=True, log=None):
        self._margin_at = margin_at
        self.arm = list(arm_names)
        self.lo = np.asarray(lo, float)
        self.hi = np.asarray(hi, float)
        self.step = max(float(check_step), 1e-3)
        self.scales = [float(s) for s in floor_scales]
        self.rrt_step = float(rrt_step)
        self.smooth_time_s = float(smooth_time_s)
        self.allow_partial = bool(allow_partial)
        self._log = log

    # ---------------------------------------------------------------- collision queries
    def margin(self, q_arm, floor_scale=1.0):
        """(min margin, pair) at one arm pose."""
        return self._margin_at(np.asarray(q_arm, float), float(floor_scale))

    def edge_margin(self, a, b, floor_scale=1.0, step=None, stop_below=None):
        """(worst margin, pair) over the straight joint segment a->b, endpoints included.

        The ramp interpolates linearly between waypoints, so this samples exactly the
        motion that will be executed. stop_below returns early on the first sample below
        it - the planner rejects far more edges than it accepts, and a rejected edge
        usually fails within a couple of samples.
        """
        a = np.asarray(a, float)
        d = np.asarray(b, float) - a
        n = int(np.ceil(float(np.max(np.abs(d))) / (self.step if step is None else step))) + 1
        ss = np.linspace(0.0, 1.0, min(max(n, 2), 400))
        # Endpoint first: it is the sample most likely to be the bad one, and the whole
        # interior is wasted work when it is.
        order = np.concatenate(([1.0, 0.0], ss[1:-1])) if len(ss) > 2 else ss[::-1]
        worst, wpair = float("inf"), ""
        for s in order:
            m, pair = self.margin(a + s * d, floor_scale)
            if m < worst:
                worst, wpair = m, pair
                if stop_below is not None and worst < stop_below:
                    break
        return worst, wpair

    # ------------------------------------------------------------------------ the search
    def plan(self, q0, home, deadline, min_scale=0, seed=0):
        """Returns (waypoints after q0, floor_scale, method, complete) or None."""
        q0 = np.asarray(q0, float)
        home = np.asarray(home, float)
        scales = self.scales[min_scale:]
        rng = np.random.default_rng(seed)
        best_partial = None
        warned = False
        for scale in scales:
            m0, p0 = self.margin(q0, scale)
            mh, _ = self.margin(home, scale)
            floor_min = min(0.0, m0)
            if m0 < 0.0 and not warned:
                warned = True
                self._warn(f"HOME: the arm is ALREADY {1000.0 * -m0:.0f} mm inside the "
                           f"{scale:.2f}x floor of {p0} - planning a path that never goes deeper")
            if mh < floor_min:
                # Not the path's fault: home itself does not clear this cushion. Only a
                # relaxation can help, so do not burn the budget searching at this one.
                continue
            for tier in (self._direct, self._staged, self._greedy, self._detour, self._rrt):
                if time.monotonic() >= deadline:
                    break
                got = tier(q0, home, scale, floor_min, deadline, rng)
                if got is None:
                    continue
                path, how = got
                if float(np.max(np.abs(np.asarray(path[-1], float) - home))) <= 1e-6:
                    return [np.asarray(w, float) for w in path], scale, how, True
                # A partial: the tier got closer but not there. Keep the best one and go on
                # - a complete path at a more relaxed cushion beats a partial at a strict one.
                gain = (float(np.max(np.abs(home - q0)))
                        - float(np.max(np.abs(home - np.asarray(path[-1], float)))))
                if gain > 0.1 and (best_partial is None or gain > best_partial[0]):
                    best_partial = (gain, [np.asarray(w, float) for w in path], scale, how)
        if best_partial is not None and self.allow_partial:
            _, path, scale, how = best_partial
            return path, scale, how, False
        return None

    def _warn(self, msg):
        if self._log is not None:
            self._log.warn(msg)

    # ------------------------------------------------------------------------- the tiers
    def _direct(self, q0, home, scale, floor_min, deadline, rng):
        w, _ = self.edge_margin(q0, home, scale, stop_below=floor_min)
        return ([np.asarray(home, float)], "direct") if w >= floor_min else None

    def _groups(self):
        """Joint groups by name, so this survives a different `arm:` list in the YAML."""
        wrist = [i for i, j in enumerate(self.arm) if "wrist" in j]
        pan = [i for i, j in enumerate(self.arm) if "pan" in j]
        upper = [i for i in range(len(self.arm)) if i not in wrist and i not in pan]
        return wrist, pan, upper

    def _staged(self, q0, home, scale, floor_min, deadline, rng):
        """Two-segment paths that take one joint group home first.

        These are the moves an operator makes by hand: fold the wrist to its home angles
        before swinging the arm, or lift the shoulder and elbow off the table before
        rotating the base. They keep the synchronized-arrival feel of the plain ramp,
        which is why they are tried before anything that moves joints one at a time.
        """
        wrist, pan, upper = self._groups()
        q0 = np.asarray(q0, float)
        for how, g in (("wrists first", wrist), ("shoulder+elbow first", upper),
                       ("arm before wrists", pan + upper), ("pan last", upper + wrist),
                       ("pan first", pan)):
            if time.monotonic() >= deadline:
                return None
            if not g or len(g) >= len(q0):
                continue
            w = q0.copy()
            for i in g:
                w[i] = home[i]
            if float(np.max(np.abs(w - q0))) < 1e-4:
                continue
            m1, _ = self.edge_margin(q0, w, scale, stop_below=floor_min)
            if m1 < floor_min:
                continue
            m2, _ = self.edge_margin(w, home, scale, stop_below=floor_min)
            if m2 >= floor_min:
                return [w, np.asarray(home, float)], how
        return None

    def _greedy(self, q0, home, scale, floor_min, deadline, rng):
        """Coordinate descent: repeatedly take the single joint move to its home value
        that leaves the most clearance, falling back to a fraction of that move when the
        whole of it collides. Six joints, so at worst six slow single-joint segments -
        ugly to watch, but it is the tier that unpicks a wrist wound into a corner."""
        q0 = np.asarray(q0, float)
        cur = q0.copy()
        path = []
        while True:
            if time.monotonic() >= deadline or len(path) > 3 * len(cur):
                return None
            left = [i for i in range(len(cur)) if abs(home[i] - cur[i]) > 1e-4]
            if not left:
                return (path, "one joint at a time") if path else None
            best = None
            for i in left:
                for frac in (1.0, 0.6, 0.3):
                    cand = cur.copy()
                    cand[i] = cur[i] + frac * (home[i] - cur[i])
                    w, _ = self.edge_margin(cur, cand, scale, stop_below=floor_min)
                    if w >= floor_min:
                        if best is None or w > best[0]:
                            best = (w, i, frac, cand)
                        break   # a bigger move on the same joint is always preferable
            if best is None:
                return None
            path.append(best[3])
            cur = best[3]

    def _detour(self, q0, home, scale, floor_min, deadline, rng):
        """Back ONE joint out first, then re-run the cheap tiers from there.

        A wrist pressed onto the table and a gripper hooked under the camera arch are both
        the same shape of problem: every direct move is blocked because the arm is inside a
        pocket, and the way out is one deliberate step BACKWARDS before any progress is
        possible. Straight-line planners cannot express that; two segments can."""
        q0 = np.asarray(q0, float)
        for delta in (np.radians(45.0), np.radians(90.0), np.radians(135.0)):
            for i in range(len(q0)):
                for sgn in (1.0, -1.0):
                    if time.monotonic() >= deadline:
                        return None
                    w = q0.copy()
                    w[i] = float(np.clip(q0[i] + sgn * delta, self.lo[i], self.hi[i]))
                    if abs(w[i] - q0[i]) < 1e-3:
                        continue
                    m, _ = self.edge_margin(q0, w, scale, stop_below=floor_min)
                    if m < floor_min:
                        continue
                    name = f"{self.arm[i]} backed out {np.degrees(w[i] - q0[i]):+.0f} deg"
                    m2, _ = self.edge_margin(w, home, scale, stop_below=floor_min)
                    if m2 >= floor_min:
                        return [w, np.asarray(home, float)], name
                    sub = self._staged(w, home, scale, floor_min, deadline, rng)
                    if sub is not None:
                        return [w] + list(sub[0]), f"{name}, then {sub[1]}"
        return None

    def _rrt(self, q0, home, scale, floor_min, deadline, rng):
        """RRT-Connect over the arm joints, then shortcut-smoothed.

        The general answer, and the reason "impossible" means "no path exists in the time
        given" rather than "the straight line was blocked". It grows two trees, one from
        the arm and one from home, and every edge it accepts is checked at the same
        resolution as the final path - so what it returns needs no re-verification.

        If the trees never meet before the deadline it still returns the branch of the
        start tree that got CLOSEST to home. That is not a failure to be discarded: the
        caller can execute it and replan from there, and a pose closer to home in joint
        space is usually a pose out of whatever pocket blocked the first search."""
        q0 = np.asarray(q0, float)
        home = np.asarray(home, float)
        lo = np.minimum(self.lo, np.minimum(q0, home))
        hi = np.maximum(self.hi, np.maximum(q0, home))
        step = self.rrt_step

        def steer(a, b):
            d = b - a
            n = float(np.max(np.abs(d)))
            return b.copy() if n <= step else a + d * (step / n)

        def ok(a, b):
            return self.edge_margin(a, b, scale, stop_below=floor_min)[0] >= floor_min

        A, PA = [q0.copy()], [-1]          # tree rooted at the arm
        B, PB = [home.copy()], [-1]        # tree rooted at home
        swapped = False

        def chain(tree, par, k):
            out = []
            while k >= 0:
                out.append(tree[k])
                k = par[k]
            return out[::-1]

        while time.monotonic() < deadline:
            qr = rng.uniform(lo, hi)
            arr = np.asarray(A)
            ia = int(np.argmin(np.max(np.abs(arr - qr), axis=1)))
            qn = steer(A[ia], qr)
            if ok(A[ia], qn):
                A.append(qn)
                PA.append(ia)
                # Greedy CONNECT from the other tree: the half of RRT-Connect that makes
                # it find paths in seconds instead of minutes.
                brr = np.asarray(B)
                k = int(np.argmin(np.max(np.abs(brr - qn), axis=1)))
                while time.monotonic() < deadline:
                    qs = steer(B[k], qn)
                    if not ok(B[k], qs):
                        break
                    B.append(qs)
                    PB.append(k)
                    k = len(B) - 1
                    if float(np.max(np.abs(qs - qn))) < 1e-9:
                        ca = chain(A, PA, len(A) - 1)
                        cb = chain(B, PB, k)[::-1][1:]
                        pts = (cb[::-1] + ca[::-1]) if swapped else (ca + cb)
                        # Smoothing gets its own clock, NOT what is left of the search
                        # deadline: an RRT that only just made it would otherwise return
                        # its raw random walk, and the raw walk is what the arm executes.
                        pts = self._shortcut(pts, scale, floor_min,
                                             time.monotonic() + self.smooth_time_s, rng)
                        return pts[1:], "rrt"
            A, PA, B, PB = B, PB, A, PA
            swapped = not swapped
        start, pstart = (B, PB) if swapped else (A, PA)
        d = np.max(np.abs(np.asarray(start) - home), axis=1)
        k = int(np.argmin(d))
        if k == 0:
            return None
        pts = self._shortcut(chain(start, pstart, k), scale, floor_min,
                             time.monotonic() + min(1.0, self.smooth_time_s), rng)
        return pts[1:], "rrt (partial)"

    def _shortcut(self, pts, scale, floor_min, deadline, rng, iters=200):
        """Drop waypoints whose neighbours can see each other. An RRT path is a random
        walk; unsmoothed it would drive the arm around the room to get across it."""
        pts = [np.asarray(p, float) for p in pts]
        for _ in range(iters):
            if len(pts) <= 2 or time.monotonic() >= deadline:
                break
            i = int(rng.integers(0, len(pts) - 2))
            j = int(rng.integers(i + 2, len(pts)))
            if self.edge_margin(pts[i], pts[j], scale, stop_below=floor_min)[0] >= floor_min:
                pts = pts[:i + 1] + pts[j:]
        return pts
