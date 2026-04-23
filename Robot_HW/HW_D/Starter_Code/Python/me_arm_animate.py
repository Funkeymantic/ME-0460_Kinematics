"""
ME 4640 – ME Arm V3.0 Animator
================================
Equivalent to the MATLAB function  animate_me_arm.m  in the IKfiles/ folder.

Animates the ME Arm V3.0 through a precomputed joint trajectory, drawing:
  • A blue dashed reference path   (the desired Cartesian trajectory)
  • A red trail                    (actual FK tool-tip position, verifies IK error)
  • Coloured 3-D boxes for each link

Dependencies
------------
  pip install numpy matplotlib

For MP4 video saving:
  Install ffmpeg and add it to PATH  (https://ffmpeg.org/download.html)
  Alternatively, GIF output works without ffmpeg.

Usage
-----
  from me_arm_animate import animate_me_arm

  # TH       : (N,3) array of [th_a, th_b, th_c] in radians
  # ref_path : (3,N) array of [x; y; z] in mm (world frame)
  animate_me_arm(TH, ref_path, label='star', save_video=False)
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from me_arm_fk import me_arm_gs0, me_arm_gs1, me_arm_gs2, me_arm_gst, rectverts


# ── Internal helper ───────────────────────────────────────────────────────────

def _face_verts(T, HV, F):
    """
    Apply 4×4 rigid-body transform T to homogeneous vertices HV and
    return a list of face-vertex arrays suitable for Poly3DCollection.

    Parameters
    ----------
    T  : (4,4) ndarray
    HV : (4,8) homogeneous vertex matrix (from rectverts)
    F  : (6,4) face-index array           (from rectverts)

    Returns
    -------
    list of 6 arrays, each (4,3) – one quad face in world coordinates
    """
    PV = (T @ HV)[:3, :]          # (3,8) world-frame vertices
    return [PV[:, face].T for face in F]   # list of (4,3) arrays


def _make_link_geometry():
    """
    Build the homogeneous vertex matrices and face arrays for all four links.
    Geometry matches the MATLAB animate_me_arm.m exactly.

    Returns  (HVs, Fs, Cs)  –  each a list of four arrays, one per link.
    """
    d2 = 80 / 1000    # upper arm (m)
    d3 = 80 / 1000    # forearm
    d4 = 60 / 1000    # tool

    # Base (red)
    wxB = 55/1000;  wyB = 60/1000;  lyfB = 18/1000
    lybB = wyB - lyfB;  hzB = 65/1000
    HV0, F0, C0 = rectverts([-0.5*wxB,  0.5*wxB,
                               -lybB,    lyfB,
                                0,       hzB],   [1, 0, 0])
    # Shoulder (green)
    wxS = 23/1000;  wyS = 6/1000;  padZ = 6/1000
    HV1, F1, C1 = rectverts([-0.5*wxS,  0.5*wxS,
                               -0.5*wyS, 0.5*wyS,
                               -padZ - d2/2, padZ + d2/2],  [0, 1, 0])
    # Elbow (blue)
    wxE = 29/1000;  wzE = 6/1000;  padY = 4/1000
    HV2, F2, C2 = rectverts([-0.5*wxE,  0.5*wxE,
                               -padY - d3/2, padY + d3/2,
                               -0.5*wzE, 0.5*wzE],           [0, 0, 1])
    # Hand/tool (magenta)
    lhX = 40/1000;  lhZup = 23/1000;  lhZdn = 15/1000;  padYh = 4/1000
    HV3, F3, C3 = rectverts([-0.5*lhX,  0.5*lhX,
                               -d4 - padYh, 0,
                               -lhZdn, lhZup],               [1, 0, 1])

    return ([HV0, HV1, HV2, HV3],
            [F0,  F1,  F2,  F3 ],
            [C0,  C1,  C2,  C3 ])


# ── Public API ────────────────────────────────────────────────────────────────

def animate_me_arm(TH, path_ref_mm, label='trajectory',
                   save_video=False, filename='me_arm_animation',
                   fps=30, speedup=1.0):
    """
    Animate the ME Arm V3.0 through a precomputed joint trajectory.

    Parameters
    ----------
    TH           : (N,3) array-like
        Joint angles [th_a, th_b, th_c] in radians, one row per frame.
        These are the raw IK outputs — no motor mapping needed here.
    path_ref_mm  : (3,N) array-like
        Desired tool-tip path in mm (world frame).  Plotted as a blue
        dashed reference; the red FK trail shows any IK error.
    label        : str
        Short description used in the window title.
    save_video   : bool
        True → save animation to disk.  MP4 if ffmpeg is installed,
        GIF otherwise.
    filename     : str
        Base output filename (no extension).
    fps          : int
        Frames per second for the saved file.
    speedup      : float
        Playback speed multiplier (1.0 = real-time at `fps`).

    Returns
    -------
    ani : matplotlib.animation.FuncAnimation
        Keep a reference to this in the caller to prevent garbage collection.

    Notes
    -----
    The red trail is computed from FK (me_arm_gst), so any gap between the
    trail and the blue reference path directly shows IK error — useful for
    validating your solver before running on hardware.
    """
    TH = np.asarray(TH, dtype=float)
    path_ref_mm = np.asarray(path_ref_mm, dtype=float)
    if path_ref_mm.ndim == 2 and path_ref_mm.shape[0] != 3:
        path_ref_mm = path_ref_mm.T   # tolerate (N,3) input
    ref_m = path_ref_mm / 1000.0      # mm → m

    HVs, Fs, Cs = _make_link_geometry()
    n_frames = len(TH)

    # ── Figure ────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(8, 8), facecolor='white')
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)');  ax.set_zlabel('Z (m)')
    ax.set_xlim(-0.05, 0.25)
    ax.set_ylim(-0.10, 0.25)
    ax.set_zlim( 0.00, 0.30)
    ax.view_init(elev=25, azim=120)
    ax.set_box_aspect([1, 1, 1])
    ax.set_title(f'ME Arm IK — {label}', fontsize=13)
    ax.grid(True)

    # Static reference path
    ax.plot(ref_m[0], ref_m[1], ref_m[2],
            'b--', lw=1.5, label='reference path')

    # FK tool-tip trail (updated each frame)
    trail, = ax.plot([], [], [], 'r-', lw=2, label='FK trail')
    ax.legend(fontsize=10, loc='upper left')

    # ── Link Poly3DCollections ────────────────────────────────────────────
    link_colors = [(*Cs[i], 0.55) for i in range(4)]   # RGBA with alpha
    edge_color  = (0, 0, 0, 0.7)

    cols = []
    for fc in link_colors:
        col = Poly3DCollection([], facecolor=fc, edgecolor=edge_color)
        ax.add_collection3d(col)
        cols.append(col)

    tool_pts = np.full((3, n_frames), np.nan)

    # ── Animation callbacks ───────────────────────────────────────────────
    def init():
        trail.set_data([], [])
        trail.set_3d_properties([])
        return (*cols, trail)

    def update(n):
        th = TH[n]
        Ts = [me_arm_gs0(th), me_arm_gs1(th), me_arm_gs2(th), me_arm_gst(th)]
        for col, T, HV, F in zip(cols, Ts, HVs, Fs):
            col.set_verts(_face_verts(T, HV, F))

        tip = (Ts[3] @ np.array([0., 0., 0., 1.]))[:3]
        tool_pts[:, n] = tip
        trail.set_data(tool_pts[0, :n+1], tool_pts[1, :n+1])
        trail.set_3d_properties(tool_pts[2, :n+1])
        return (*cols, trail)

    interval_ms = max(1, int(1000 / fps / speedup))
    ani = animation.FuncAnimation(
        fig, update, frames=n_frames,
        init_func=init, interval=interval_ms,
        blit=False, repeat=False
    )

    # ── Save ──────────────────────────────────────────────────────────────
    if save_video:
        try:
            writer = animation.FFMpegWriter(fps=fps,
                                            extra_args=['-vcodec', 'libx264'])
            ani.save(filename + '.mp4', writer=writer, dpi=120)
            print(f"Saved {filename}.mp4")
        except Exception as exc:
            print(f"FFMpeg unavailable ({exc}). Saving GIF instead.")
            print("Install ffmpeg (https://ffmpeg.org) for MP4 output.")
            writer = animation.PillowWriter(fps=fps)
            ani.save(filename + '.gif', writer=writer, dpi=100)
            print(f"Saved {filename}.gif")

    plt.tight_layout()
    plt.show()
    return ani   # caller must keep a reference to prevent GC
