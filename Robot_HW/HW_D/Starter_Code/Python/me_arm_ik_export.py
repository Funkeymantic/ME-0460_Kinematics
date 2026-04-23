"""
ME 4640 – HW D Part 4  |  IK Trajectory → Arduino Header Export
=================================================================
Python equivalent of  ME_Arm_IK_Export.m.

This script:
  1. Generates a Cartesian trajectory (replace the star example with
     your own Part 2 path).
  2. Runs the IK solver on every interpolated waypoint.
  3. Maps IK angles (th_a, th_b, th_c) → motor angles (th1, th2, th3).
  4. Exports  trajectory.h  for use with  HWD_IK_Trajectory.ino.
  5. (Optional) plots the motor angles so you can verify before uploading.

Motor-angle mapping  (from Robot_HWC_Soln.m):
    th1 =  th_a
    th2 =  th_b
    th3 = -(th_c + th_b)

Usage
-----
  python me_arm_ik_export.py
  # → writes trajectory.h in the current working directory

Copy  trajectory.h  into the same folder as  HWD_IK_Trajectory.ino
before uploading to the Arduino.

Dependencies
------------
  pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt

from me_arm_ik import MEarmIK, ik_to_motor_deg


# =============================================================================
#  PART 2 TRAJECTORY  –  Replace this block with your own path generator.
#  Your function should return arrays  px, py, pz  (world-frame, mm),
#  one entry per interpolated waypoint.
# =============================================================================

def build_star_trajectory():
    """Example: 5-pointed star. Replace with your own shape."""
    base_angle = np.pi / 2      # arm points along +Y
    cr_mm = 110.;  cz_mm = 130. # star centre (radial, height) in mm
    r_outer = 35.;  r_inner = 15.  # star radii in mm
    n_tips = 5

    ang_outer = np.pi/2 + np.arange(n_tips) * (2*np.pi / n_tips)
    ang_inner = ang_outer + np.pi / n_tips

    angs = np.empty(2 * n_tips)
    angs[0::2] = ang_outer
    angs[1::2] = ang_inner
    r = np.empty(2 * n_tips)
    r[0::2] = r_outer
    r[1::2] = r_inner

    star_rad = cr_mm + r * np.cos(angs)   # radial component
    star_z   = cz_mm + r * np.sin(angs)   # world Z (vertical)
    star_rad = np.append(star_rad, star_rad[0])  # close loop
    star_z   = np.append(star_z,   star_z[0])

    star_x = star_rad * np.cos(base_angle)
    star_y = star_rad * np.sin(base_angle)

    steps_per_seg = 20
    px, py, pz = [], [], []
    for k in range(len(star_x) - 1):
        px.append(np.linspace(star_x[k], star_x[k+1], steps_per_seg))
        py.append(np.linspace(star_y[k], star_y[k+1], steps_per_seg))
        pz.append(np.linspace(star_z[k], star_z[k+1], steps_per_seg))

    return np.concatenate(px), np.concatenate(py), np.concatenate(pz)


# =============================================================================
#  MAIN EXPORT WORKFLOW
# =============================================================================

def main():
    # ── Build trajectory ──────────────────────────────────────────────────
    # Replace  build_star_trajectory()  with your own function from Part 2.
    px, py, pz = build_star_trajectory()
    N = len(px)
    print(f"Trajectory has {N} waypoints.")

    # ── Solve IK ─────────────────────────────────────────────────────────
    th_a = np.zeros(N)
    th_b = np.zeros(N)
    th_c = np.zeros(N)
    for k in range(N):
        th_a[k], th_b[k], th_c[k] = MEarmIK(px[k], py[k], pz[k])

    # ── Motor-angle mapping (radians → degrees) ──────────────────────────
    #   th1 =  th_a
    #   th2 =  th_b
    #   th3 = -(th_c + th_b)   [from Robot_HWC_Soln.m FK derivation]
    th1_deg, th2_deg, th3_deg = ik_to_motor_deg(th_a, th_b, th_c)
    TRAJ_deg = np.column_stack([th1_deg, th2_deg, th3_deg])   # (N, 3)

    # ── Export trajectory.h ───────────────────────────────────────────────
    header_file = 'trajectory.h'
    export_trajectory_header(TRAJ_deg, header_file)

    # ── Optional: plot motor angles for visual verification ───────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    titles = [r'$\theta_1$ — base', r'$\theta_2$ — shoulder', r'$\theta_3$ — elbow']
    for i, (ax, title) in enumerate(zip(axes, titles)):
        ax.plot(TRAJ_deg[:, i])
        ax.set_ylabel('deg')
        ax.set_title(title)
        ax.grid(True)
    axes[-1].set_xlabel('Waypoint index')
    fig.suptitle('Exported motor angles — verify before uploading to Arduino',
                 fontsize=12)
    plt.tight_layout()
    plt.show()


# =============================================================================
#  EXPORT FUNCTION
# =============================================================================

def export_trajectory_header(TRAJ_deg, filename='trajectory.h'):
    """
    Write a C header file for the HWD_IK_Trajectory Arduino sketch.

    The array is stored in AVR program memory (PROGMEM) to keep it out of
    the Arduino Uno's limited 2 KB SRAM.  Flash usage:
        N waypoints × 3 joints × 4 bytes = N×12 bytes
    (200 waypoints ≈ 2.3 KB flash — well within the Uno's 32 KB limit.)

    Parameters
    ----------
    TRAJ_deg : (N, 3) ndarray
        Motor angles [th1, th2, th3] in home-relative degrees.
    filename : str
        Output filename.  Place the result in the Arduino sketch folder.
    """
    N = TRAJ_deg.shape[0]
    lines = []

    lines += [
        '// trajectory.h  –  auto-generated by me_arm_ik_export.py',
        '// DO NOT EDIT MANUALLY – regenerate from your IK script.',
        '//',
        '// Columns: { th1_deg, th2_deg, th3_deg }',
        '//   th1 = base     (home-relative, degrees)',
        '//   th2 = shoulder (home-relative, degrees)',
        '//   th3 = elbow    (home-relative, degrees)',
        '//',
        '// Mapping from IK output (th_a, th_b, th_c) to motor angles:',
        '//   th1 =  th_a',
        '//   th2 =  th_b',
        '//   th3 = -(th_c + th_b)',
        '',
        '#pragma once',
        '#include <avr/pgmspace.h>',
        '',
        f'#define N_WPT {N}',
        '',
        f'const float TRAJ[N_WPT][3] PROGMEM = {{',
    ]

    for k, row in enumerate(TRAJ_deg):
        comma = ',' if k < N - 1 else ' '
        lines.append(f'  {{ {row[0]:9.4f}, {row[1]:9.4f}, {row[2]:9.4f} }}{comma}')

    lines += ['};', '']

    with open(filename, 'w') as f:
        f.write('\n'.join(lines))

    flash_kb = N * 3 * 4 / 1024
    print(f'\nExported  {filename}')
    print(f'  Waypoints  : {N}')
    print(f'  Flash used : {flash_kb:.1f} KB  (of 32 KB on Uno)')
    print(f'\nNext step: copy {filename} into your HWD_IK_Trajectory sketch folder.')


if __name__ == '__main__':
    main()
