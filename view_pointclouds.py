#!/usr/bin/env python3
"""Render or explore the point clouds built by make_pointclouds.py, to eyeball the calibration.

Draws each frame's per-camera clouds in the robot base frame with a frame axis at the base and
a grid on the z = 0 plane.

  view_pointclouds.py [dataset name]                 offscreen PNGs, two views per frame
  view_pointclouds.py [dataset name] --interactive   a window you can fly around and measure in

Offscreen mode writes an iso/side/top triple per frame, twice: the clouds in their own colours,
and the same clouds tinted per camera (left red / right blue) so misalignment shows up directly.
Interactive mode is the same scene under an orbit camera, plus frame stepping, per-camera
toggles, a side-by-side split of the two cameras, a z-slice, an on-demand calibration readout
and point picking. Press H in the window for the key map.
"""
import copy
import sys
from pathlib import Path

import numpy as np

SHARE = Path("/home/ros/share")
try:
    import open3d as o3d
except ModuleNotFoundError:  # open3d lives in the vendored prefix, not in site-packages
    sys.path.insert(0, str(SHARE / "o3d_libs"))
    import open3d as o3d

PCD = SHARE / "pointclouds"
OUT = SHARE / "pointclouds" / "views"
CAM_TINT = {"left": (0.90, 0.25, 0.25), "right": (0.25, 0.45, 0.95)}

# Views as (front, up, zoom) for the orbit camera; front points from the target toward the eye.
PRESETS = {
    "iso":  ([0.5, -1.0, 0.6], [0.0, 0.0, 1.0], 0.7),
    "side": ([0.0, -1.0, 0.12], [0.0, 0.0, 1.0], 0.7),
    "top":  ([0.0, -0.05, 1.0], [0.0, 1.0, 0.0], 0.7),
}


def grid_on_floor(size=1.6, step=0.2, z=0.0):
    """LineSet grid on the z plane, so the table height is easy to judge."""
    pts, lines = [], []
    n = int(size / step)
    for i in range(-n, n + 1):
        x = i * step
        pts += [[x, -size, z], [x, size, z], [-size, x, z], [size, x, z]]
        lines += [[len(pts) - 4, len(pts) - 3], [len(pts) - 2, len(pts) - 1]]
    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(np.array(pts)),
                              o3d.utility.Vector2iVector(np.array(lines)))
    ls.colors = o3d.utility.Vector3dVector(np.tile([0.6, 0.6, 0.6], (len(lines), 1)))
    return ls


def load_frame(name, f_i, merged=False):
    """{camera -> PointCloud} for one exported frame. `merged` adds the combined cloud."""
    out = {}
    for cam in (("left", "right", "merged") if merged else ("left", "right")):
        p = PCD / f"{name}_f{f_i}_{cam}.ply"
        if p.exists():
            out[cam] = o3d.io.read_point_cloud(str(p))
    return out


def frame_indices(name):
    return sorted({int(p.stem.split("_f")[1].split("_")[0])
                   for p in PCD.glob(f"{name}_f*_merged.ply")})


def render(geoms, path, eye_dir, size=(1600, 1000)):
    """Offscreen render (Filament/EGL) to PNG; returns False if no renderer is available."""
    try:
        w, h = size
        r = o3d.visualization.rendering.OffscreenRenderer(w, h)
        r.scene.set_background([0.08, 0.08, 0.10, 1.0])
        pcd_mat = o3d.visualization.rendering.MaterialRecord()
        pcd_mat.shader = "defaultUnlit"
        pcd_mat.point_size = 2.0
        line_mat = o3d.visualization.rendering.MaterialRecord()
        line_mat.shader = "unlitLine"
        line_mat.line_width = 1.0
        mesh_mat = o3d.visualization.rendering.MaterialRecord()
        mesh_mat.shader = "defaultLit"
        lo, hi = None, None
        for i, g in enumerate(geoms):
            if isinstance(g, o3d.geometry.PointCloud):
                mat = pcd_mat
                bb = g.get_axis_aligned_bounding_box()
                mn, mx = np.asarray(bb.min_bound), np.asarray(bb.max_bound)
                lo = mn if lo is None else np.minimum(lo, mn)
                hi = mx if hi is None else np.maximum(hi, mx)
            elif isinstance(g, o3d.geometry.LineSet):
                mat = line_mat
            else:
                mat = mesh_mat
            r.scene.add_geometry(f"g{i}", g, mat)
        center = (lo + hi) / 2.0 if lo is not None else np.zeros(3)
        extent = float(np.linalg.norm(hi - lo)) if lo is not None else 2.0
        eye = center + np.asarray(eye_dir, float) / np.linalg.norm(eye_dir) * extent * 0.9
        r.setup_camera(60.0, center, eye, [0.0, 0.0, 1.0])
        o3d.io.write_image(str(path), r.render_to_image())
        return True
    except Exception as exc:
        print(f"  offscreen render unavailable ({exc})")
        return False


HELP = """
 view      mouse drag orbit | ctrl+drag pan | wheel zoom
 frame     N next   P prev
 clouds    1 left   2 right   3 merged   T tint per camera on/off
 split     S side-by-side on/off   , closer   . further apart
 scene     G grid   A axes    +/- point size
 camera    Z top    X side    C iso    R refit to the cloud
 z-slice   \\ on/off   [ down   ] up   ; thinner   ' thicker
 analyse   I calibration readout for this frame (stdout)
           M picking window: shift+click points, close it for coords + distances
 H help    Q quit
"""


class Viewer:
    """Interactive scene over one dataset's exported frames.

    Everything is rebuilt into the same window rather than opening a new one per frame, so the
    camera pose survives frame stepping - which is the whole point when you are comparing where
    the two cameras put the same table across frames.
    """

    def __init__(self, name, frames):
        self.name = name
        self.frames = frames
        self.i = 0
        self.show = {"left": True, "right": True, "merged": False}
        self.tint = False
        self.grid_on = True
        self.axes_on = True
        self.slice_on = False
        self.z_lo, self.z_thick = -0.10, 0.35
        self.split = False
        self.gap_scale = 1.05
        self.cache = {}
        self._gap = {}
        self.grid = grid_on_floor()
        self.axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25)
        self.vis = o3d.visualization.VisualizerWithKeyCallback()

    # ---- data -------------------------------------------------------------
    def cams(self):
        f_i = self.frames[self.i]
        if f_i not in self.cache:
            self.cache[f_i] = load_frame(self.name, f_i, merged=True)
        return self.cache[f_i]

    def offsets(self):
        """Per-camera translation for the split view: the two clouds pushed apart along y.

        The gap is the wider cloud's own y-span, so the two never overlap however you orbit,
        and it is derived per frame rather than fixed - a frame where one camera sees far less
        should not be drawn a scene apart from the other.
        """
        if not self.split:
            return {k: np.zeros(3) for k in ("left", "right", "merged")}
        f_i = self.frames[self.i]
        if f_i not in self._gap:
            cams = self.cams()
            spans = [float(np.ptp(np.asarray(cams[k].points)[:, 1]))
                     for k in ("left", "right") if k in cams]
            self._gap[f_i] = max(spans) if spans else 2.0
        g = self._gap[f_i] * self.gap_scale
        return {"left": np.array([0.0, -g / 2, 0.0]),
                "right": np.array([0.0, +g / 2, 0.0]),
                "merged": np.zeros(3)}

    def visible(self, split=None):
        """The clouds to draw, already tinted, z-sliced and (in split view) pushed apart.

        'merged' hides the per-camera pair. `split=False` forces the true base-frame position,
        which is what picking needs - a coordinate read off a shifted cloud is a lie.
        """
        cams = self.cams()
        keys = ["merged"] if self.show["merged"] else [k for k in ("left", "right") if self.show[k]]
        off = self.offsets() if (self.split if split is None else split) else None
        out = {}
        for k in keys:
            pc = cams.get(k)
            if pc is None:
                continue
            if self.tint and k in CAM_TINT:
                pc = o3d.geometry.PointCloud(pc)
                pc.paint_uniform_color(CAM_TINT[k])
            if self.slice_on:
                box = o3d.geometry.AxisAlignedBoundingBox(
                    (-1e3, -1e3, self.z_lo), (1e3, 1e3, self.z_lo + self.z_thick))
                pc = pc.crop(box)
            if off is not None and np.any(off[k]):
                pc = o3d.geometry.PointCloud(pc).translate(off[k])
            out[k] = pc
        return out

    # ---- rendering --------------------------------------------------------
    def redraw(self, refit=False):
        self.vis.clear_geometries()
        shown = self.visible()
        for pc in shown.values():
            self.vis.add_geometry(pc, reset_bounding_box=refit)
        # In split view each half gets its own grid and axes, so "how far above z=0 is this
        # camera's table" stays answerable per side instead of only against a shared origin.
        off = self.offsets()
        marks = [off[k] for k in shown] if self.split else [np.zeros(3)]
        for m in marks:
            if self.grid_on:
                self.vis.add_geometry(copy.deepcopy(self.grid).translate(m),
                                      reset_bounding_box=False)
            if self.axes_on:
                self.vis.add_geometry(copy.deepcopy(self.axes).translate(m),
                                      reset_bounding_box=False)
        self.status()
        return True

    def status(self):
        shown = self.visible()
        npts = sum(len(np.asarray(p.points)) for p in shown.values())
        sl = (f"  z[{self.z_lo:+.2f},{self.z_lo + self.z_thick:+.2f}]" if self.slice_on else "")
        sl += "  SPLIT" if self.split else ""
        title = (f"{self.name}  frame {self.frames[self.i]} "
                 f"({self.i + 1}/{len(self.frames)})  {'+'.join(shown) or 'nothing'}  "
                 f"{npts} pts{'  tinted' if self.tint else ''}{sl}")
        print(f"\r{title}   ", end="", flush=True)

    def view(self, preset):
        front, up, zoom = PRESETS[preset]
        vc = self.vis.get_view_control()
        shown = self.visible()
        if shown:
            allpts = np.vstack([np.asarray(p.points) for p in shown.values()])
            vc.set_lookat(allpts.mean(axis=0))
        vc.set_front(front)
        vc.set_up(up)
        vc.set_zoom(zoom)
        return True

    # ---- analysis ---------------------------------------------------------
    def analyse(self):
        """Print the same numbers check_calibration.py reports, for the frame on screen.

        Imported from that module rather than reimplemented, so the window and the batch check
        can never disagree about what 'the cameras agree' means.
        """
        sys.path.insert(0, str(SHARE))
        import check_calibration as CC

        cams = self.cams()
        f_i = self.frames[self.i]
        print(f"\n--- {self.name} frame {f_i} ---")
        for k in ("left", "right"):
            pc = cams.get(k)
            if pc is None:
                print(f"  {k:5s}: missing")
                continue
            pts = np.asarray(pc.points)
            h, tilt, n_in = CC.table_plane(pc)
            lo, hi = pts.min(0), pts.max(0)
            print(f"  {k:5s}: {len(pts):7d} pts   "
                  f"x[{lo[0]:+.2f},{hi[0]:+.2f}] y[{lo[1]:+.2f},{hi[1]:+.2f}] z[{lo[2]:+.2f},{hi[2]:+.2f}]")
            print(f"         table plane: height {h * 1000:+.1f} mm above base z=0, "
                  f"tilt {tilt:.1f} deg, {n_in} inliers")
        if "left" in cams and "right" in cams:
            n, p50, p95 = CC.overlap_distance(cams["left"], cams["right"])
            print(f"  left->right over the shared volume: n={n}, p50 {p50:.1f} mm, p95 {p95:.1f} mm")
            print("  (p50 under ~30 mm and |height| under ~50 mm is what check_calibration calls consistent)")
        return False

    def pick(self):
        """Second window in Open3D's editing mode: shift+click points, close it to get them back.

        A separate window because the legacy visualizer cannot have key callbacks and vertex
        picking at once. Picks come back as indices into the cloud that was handed over.
        """
        shown = self.visible(split=False)
        if not shown:
            print("\n  nothing visible to pick from")
            return False
        merged = o3d.geometry.PointCloud()
        for pc in shown.values():
            merged += pc
        print("\n  picking window: shift+click points, then close the window (Q)")
        ed = o3d.visualization.VisualizerWithEditing()
        ed.create_window(window_name=f"{self.name} f{self.frames[self.i]} - shift+click to pick",
                         width=1200, height=800)
        ed.add_geometry(merged)
        ed.run()
        ed.destroy_window()
        idx = ed.get_picked_points()
        pts = np.asarray(merged.points)[idx] if len(idx) else np.empty((0, 3))
        if not len(pts):
            print("  no points picked")
            return False
        for n, p in enumerate(pts):
            print(f"  p{n}: [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}] m")
        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                d = pts[b] - pts[a]
                print(f"  |p{a}-p{b}| = {np.linalg.norm(d) * 1000:7.1f} mm   "
                      f"dx {d[0] * 1000:+7.1f}  dy {d[1] * 1000:+7.1f}  dz {d[2] * 1000:+7.1f}")
        return False

    # ---- keys -------------------------------------------------------------
    def step(self, d):
        self.i = (self.i + d) % len(self.frames)
        return self.redraw()

    def toggle(self, key):
        self.show[key] = not self.show[key]
        return self.redraw()

    def point_size(self, d):
        ro = self.vis.get_render_option()
        ro.point_size = float(np.clip(ro.point_size + d, 1.0, 12.0))
        return True

    def slide(self, dz=0.0, dt=0.0):
        self.z_lo += dz
        self.z_thick = float(np.clip(self.z_thick + dt, 0.02, 4.0))
        if not self.slice_on:
            self.slice_on = True
        return self.redraw()

    def spread(self, d):
        self.gap_scale = float(np.clip(self.gap_scale + d, 0.0, 6.0))
        if not self.split:
            self.split = True
        return self.redraw()

    def run(self):
        self.vis.create_window(window_name=f"{self.name} - point clouds", width=1600, height=1000)
        ro = self.vis.get_render_option()
        ro.background_color = np.array([0.08, 0.08, 0.10])
        ro.point_size = 2.0
        ro.show_coordinate_frame = False
        k = self.vis.register_key_callback
        k(ord("N"), lambda v: self.step(+1))
        k(ord("P"), lambda v: self.step(-1))
        k(ord("1"), lambda v: self.toggle("left"))
        k(ord("2"), lambda v: self.toggle("right"))
        k(ord("3"), lambda v: self.toggle("merged"))
        k(ord("T"), lambda v: (setattr(self, "tint", not self.tint), self.redraw())[1])
        k(ord("G"), lambda v: (setattr(self, "grid_on", not self.grid_on), self.redraw())[1])
        k(ord("A"), lambda v: (setattr(self, "axes_on", not self.axes_on), self.redraw())[1])
        k(ord("="), lambda v: self.point_size(+1))
        k(ord("-"), lambda v: self.point_size(-1))
        k(ord("Z"), lambda v: self.view("top"))
        k(ord("X"), lambda v: self.view("side"))
        k(ord("C"), lambda v: self.view("iso"))
        k(ord("R"), lambda v: self.redraw(refit=True))
        k(ord("\\"), lambda v: (setattr(self, "slice_on", not self.slice_on), self.redraw())[1])
        k(ord("["), lambda v: self.slide(dz=-0.02))
        k(ord("]"), lambda v: self.slide(dz=+0.02))
        k(ord(";"), lambda v: self.slide(dt=-0.02))
        k(ord("'"), lambda v: self.slide(dt=+0.02))
        k(ord("S"), lambda v: (setattr(self, "split", not self.split), self.redraw(refit=True))[1])
        k(ord(","), lambda v: self.spread(-0.15))
        k(ord("."), lambda v: self.spread(+0.15))
        k(ord("I"), lambda v: self.analyse())
        k(ord("M"), lambda v: self.pick())
        k(ord("H"), lambda v: (print(HELP), False)[1])
        self.redraw(refit=True)
        self.view("iso")
        print(HELP)
        self.vis.run()
        self.vis.destroy_window()
        print()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    name = args[0] if args else "stack_three_cups"
    interactive = "--interactive" in sys.argv
    frames = frame_indices(name)
    if not frames:
        print(f"no exported clouds for {name} in {PCD}; run make_pointclouds.py first")
        return 1

    if interactive:
        Viewer(name, frames).run()
        return 0

    OUT.mkdir(parents=True, exist_ok=True)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25)
    grid = grid_on_floor()
    for f_i in frames:
        cams = load_frame(name, f_i)
        if not cams:
            continue
        natural = list(cams.values())
        tinted = []
        for cam, pc in cams.items():
            t = o3d.geometry.PointCloud(pc)
            t.paint_uniform_color(CAM_TINT[cam])
            tinted.append(t)
        npts = sum(len(c.points) for c in natural)
        views = {"iso": [0.5, -1.0, 0.6], "side": [0.0, -1.0, 0.12], "top": [0.0, -0.05, 1.0]}
        for vname, eye_dir in views.items():
            ok = render(natural + [axes, grid], OUT / f"{name}_f{f_i}_{vname}_rgb.png", eye_dir)
            render(tinted + [axes, grid], OUT / f"{name}_f{f_i}_{vname}_percam.png", eye_dir)
            if not ok:
                return 2
        print(f"  frame {f_i}: {npts} points -> {OUT}/{name}_f{f_i}_*.png")
    print(f"\nviews in {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
