"""
ME 4640 – ME Arm V3.0 IK Test Script
======================================
Python equivalent of  ME_Arm_IK_Test.m  (MATLAB/ME_Arm_IK_Test.m).

Demonstrates the full IK workflow in five explicit steps so the
coordinate-frame story is visible at every stage:

  1. Build the desired shape in its own LOCAL frame (flat in the XY plane).
  2. Visualise the shape in its local frame (Figure 1).
  3. Build T_world (4×4 homogeneous transform) and map every point to
     world coordinates.
  4. Visualise the result in the world frame (Figure 2).
  5. Solve IK for every interpolated waypoint, then animate the arm.

Switch trajectories by editing the ONE uncommented shape call in main().

Dependencies
------------
  pip install numpy matplotlib
  + SO3.py / SE3.py / robotics_utils.py  (course utilities, same folder)

For MP4 video saving install ffmpeg and add to PATH (https://ffmpeg.org).
GIF output works without ffmpeg.

Usage
-----
  python me_arm_ik_test.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # registers 3D projection

import SE3                                 # course SE3 utilities
from me_arm_ik      import MEarmIK
from me_arm_animate import animate_me_arm


# =============================================================================
#  TRAJECTORY SHAPE BUILDERS
#  Each returns  (pts, label):
#    pts   – (3, N) float ndarray in the local frame, Z = 0, units: mm
#    label – short string used in titles and filenames
# =============================================================================

def build_star(radii, n_tips=5):
    """
    n-pointed star in the local XY plane, centred at the origin.

    Parameters
    ----------
    radii  : [r_outer, r_inner]  mm
    n_tips : int – number of star points (5 for a standard star)

    Returns
    -------
    pts   : (3, 2*n_tips+1) float ndarray  (Z = 0, path closed)
    label : 'star'
    """
    r_outer, r_inner = radii
    ang_outer = np.pi/2 + np.arange(n_tips) * (2*np.pi / n_tips)   # first tip at +Y
    ang_inner = ang_outer + np.pi / n_tips

    angs = np.empty(2 * n_tips);  angs[0::2] = ang_outer;  angs[1::2] = ang_inner
    r    = np.empty(2 * n_tips);  r[0::2]    = r_outer;    r[1::2]    = r_inner

    x = r * np.cos(angs);  y = r * np.sin(angs);  z = np.zeros_like(x)

    # Close the path (append first point)
    return np.vstack([np.append(x, x[0]),
                      np.append(y, y[0]),
                      np.append(z, z[0])]), 'star'


def build_moon(R, r_in, dx_in):
    """
    Crescent moon in the local XY plane, centred at origin.

    Parameters
    ----------
    R     : outer (convex back) radius, mm
    r_in  : inner (concave front) radius, mm
    dx_in : inner circle X-offset, mm  (controls crescent thickness)

    Returns
    -------
    pts   : (3, N) float ndarray  (Z = 0)
    label : 'moon'
    """
    dx_tip = (R**2 - r_in**2 + dx_in**2) / (2 * dx_in)
    dy_tip = np.sqrt(max(R**2 - dx_tip**2, 0.0))

    phi_top = np.arctan2( dy_tip,  dx_tip)
    phi_bot = np.arctan2(-dy_tip,  dx_tip)
    psi_top = np.arctan2( dy_tip,  dx_tip - dx_in)
    psi_bot = np.arctan2(-dy_tip,  dx_tip - dx_in)

    # Outer arc: top tip → CCW → bottom tip  (convex back)
    ao = np.linspace(phi_top, phi_bot + 2*np.pi, 60)
    xo = R * np.cos(ao);  yo = R * np.sin(ao)

    # Inner arc: bottom tip → CW → top tip  (concave front)
    ai = np.linspace(psi_bot, psi_top - 2*np.pi, 40)
    xi = dx_in + r_in * np.cos(ai);  yi = r_in * np.sin(ai)

    x = np.concatenate([xo, xi, [xo[0]]])
    y = np.concatenate([yo, yi, [yo[0]]])
    z = np.zeros_like(x)
    return np.vstack([x, y, z]), 'moon'


# =============================================================================
#  COORDINATE FRAME UTILITIES
# =============================================================================

def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], dtype=float)

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], dtype=float)

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)


def plane_rot(plane_str):
    """
    4×4 rotation that maps the canonical local frame
    (shape flat in XY, first tip along +Y_local, normal = +Z_local)
    into the requested world plane.

      'xy'  →  no rotation
      'xz'  →  rot_x(-90°)   (Y_local → +Z_world)
      'yz'  →  rot_y(90°) @ rot_z(90°)
    """
    p = plane_str.lower()
    if   p == 'xy':  return np.eye(4)
    elif p == 'xz':  return rot_x(-np.pi / 2)
    elif p == 'yz':  return rot_y(np.pi/2) @ rot_z(np.pi/2)
    else: raise ValueError(f"Unknown plane '{plane_str}'. Use 'xy', 'xz', or 'yz'.")


def build_T_world(center_xyz, plane_str):
    """
    Assemble the 4×4 world transform:
        T_world = plane_rotation  (with translation = center_xyz)

    Parameters
    ----------
    center_xyz : (3,) array – world-frame shape centre in mm
    plane_str  : 'xy' | 'xz' | 'yz'

    Returns
    -------
    T : (4,4) ndarray
    """
    T = plane_rot(plane_str)
    T[:3, 3] = np.asarray(center_xyz, dtype=float)
    return T


# =============================================================================
#  WAYPOINT INTERPOLATION
# =============================================================================

def interp_waypoints(waypts, steps_per_seg):
    """
    Linearly interpolate between successive columns of waypts.

    Parameters
    ----------
    waypts       : (3, M) float ndarray – world-frame waypoints (mm)
    steps_per_seg: int – interpolated points per segment

    Returns
    -------
    px, py, pz : 1-D ndarrays  (length = (M-1)*steps_per_seg)
    """
    px, py, pz = [], [], []
    for k in range(waypts.shape[1] - 1):
        px.append(np.linspace(waypts[0,k], waypts[0,k+1], steps_per_seg))
        py.append(np.linspace(waypts[1,k], waypts[1,k+1], steps_per_seg))
        pz.append(np.linspace(waypts[2,k], waypts[2,k+1], steps_per_seg))
    return np.concatenate(px), np.concatenate(py), np.concatenate(pz)


# =============================================================================
#  VISUALISATION HELPERS
# =============================================================================

def plot_local_frame(pts, label, plane_str):
    """Figure 1 – shape in its local frame before any world transform."""
    fig = plt.figure(figsize=(6, 6), facecolor='white',
                     num=f'Step 1 – "{label}" in local frame')
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pts[0], pts[1], pts[2], 'b-o', lw=2, ms=4)
    ax.plot(0, 0, 0, 'k+', ms=14, mew=2)

    sc = float(np.max(np.abs(pts))) * 0.45 or 20.
    for vec, lbl, col in zip(np.eye(3)*sc,
                              ['x_local','y_local','z_local'],
                              ['r',[0,0.6,0],'b']):
        ax.quiver(0,0,0,*vec, color=col, arrow_length_ratio=0.2)
        ax.text(*(vec*1.15), lbl, color=col, fontsize=10)

    ax.set_xlabel('u (mm)');  ax.set_ylabel('v (mm)');  ax.set_zlabel('w (mm)')
    ax.set_title(f'"{label}" — local frame\n(maps into the {plane_str.upper()} plane)',
                 fontsize=12)
    ax.grid(True);  plt.tight_layout();  plt.show(block=False)


def plot_world_frame(pts_mm, center_xyz, plane_str, label):
    """Figure 2 – shape in world frame after T_world is applied."""
    fig = plt.figure(figsize=(7, 6), facecolor='white',
                     num=f'Step 2 – "{label}" in world frame ({plane_str})')
    ax = fig.add_subplot(111, projection='3d')

    sc = 60   # mm
    for vec, lbl, col in zip(np.eye(3)*sc,
                              ['X_W','Y_W','Z_W'],
                              ['r',[0,0.6,0],'b']):
        ax.quiver(0,0,0,*vec, color=col, arrow_length_ratio=0.15)
        ax.text(*(vec*1.15), lbl, color=col, fontsize=10)

    ax.plot(pts_mm[0], pts_mm[1], pts_mm[2], 'b-o', lw=2, ms=4)
    ax.plot(*center_xyz, 'k+', ms=14, mew=2)
    ax.set_xlabel('X (mm)');  ax.set_ylabel('Y (mm)');  ax.set_zlabel('Z (mm)')
    c = center_xyz
    ax.set_title(f'"{label}" after T_world\n'
                 f'plane={plane_str.upper()},  centre=[{c[0]:.0f} {c[1]:.0f} {c[2]:.0f}] mm',
                 fontsize=11)
    ax.view_init(elev=25, azim=120);  ax.grid(True)
    plt.tight_layout();  plt.show(block=False)


# =============================================================================
#  MAIN WORKFLOW
# =============================================================================

def main():
    # ── Choose trajectory ─────────────────────────────────────────────────
    # Uncomment ONE shape.  Edit center_xyz and plane_str to move / reorient.

    waypts_local, label = build_star([35, 15], n_tips=5)   # [r_outer, r_inner]
    # waypts_local, label = build_moon(35, 28, 20)          # R, r_inner, dx_offset

    center_xyz = np.array([140., 150., 130.])   # mm — world-frame shape centre
    plane_str  = 'xz'                            # 'xy' | 'xz' | 'yz'
    # ─────────────────────────────────────────────────────────────────────

    # Step 1 — local frame visualisation
    plot_local_frame(waypts_local, label, plane_str)

    # Step 2 — map to world frame
    T_world     = build_T_world(center_xyz, plane_str)
    n_local     = waypts_local.shape[1]
    pts_h       = T_world @ np.vstack([waypts_local, np.ones((1, n_local))])
    waypts_world = pts_h[:3, :]   # (3, N)

    # Step 3 — world frame visualisation
    plot_world_frame(waypts_world, center_xyz, plane_str, label)

    # Step 4 — interpolate & solve IK
    steps_per_seg = 20
    px, py, pz    = interp_waypoints(waypts_world, steps_per_seg)
    n_frames      = len(px)

    TH = np.zeros((n_frames, 3))
    for k in range(n_frames):
        th_a, th_b, th_c = MEarmIK(px[k], py[k], pz[k])
        TH[k] = [th_a, th_b, th_c]

    # Step 5 — animate
    record_video = False   # True → save MP4 (needs ffmpeg) or GIF
    ref_path_mm  = np.vstack([px, py, pz])   # (3, N)

    ani = animate_me_arm(TH, ref_path_mm, label=label,
                         save_video=record_video,
                         filename=f'me_arm_{label}')
    return ani   # keep reference alive to prevent GC


if __name__ == '__main__':
    ani = main()
    plt.show()   # block until all windows are closed
