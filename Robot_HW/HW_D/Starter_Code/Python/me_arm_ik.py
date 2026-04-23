"""
ME 4640 – ME Arm V3.0 Inverse Kinematics
==========================================
Closed-form IK for the ME Arm V3.0 using direct geometric analysis.
Equivalent to the MATLAB function MEarmIK.m.

Background
----------
The ME Arm is a 3-DOF serial chain with one base rotation (th_a) and two
planar links (th_b, th_c) in the arm's reach-elevation plane.  The wrist
joint is kinematically constrained (th4 = -th_b - th_c) so it is not an
independent degree of freedom.

Angle conventions  (match MEarmIK.m exactly)
---------------------------------------------
  th_a  – base angle     (rad, measured from +Y axis)
  th_b  – shoulder angle (rad, pi/2 = arm vertical)
  th_c  – elbow angle    (rad,   0  = arm straight)

Motor-angle mapping  (from Robot_HWC_Soln.m, needed for Arduino export)
------------------------------------------------------------------------
  th1 =  th_a
  th2 =  th_b
  th3 = -(th_c + th_b)

Usage
-----
  from me_arm_ik import MEarmIK
  import numpy as np

  th_a, th_b, th_c = MEarmIK(140, 110, 130)   # mm
"""

import numpy as np
import warnings


def MEarmIK(x, y, z):
    """
    Closed-form inverse kinematics for the ME Arm V3.0.

    Parameters
    ----------
    x, y, z : float
        Desired tool-tip position in the WORLD frame (mm).
        World-frame origin is at the base foot; the shoulder pivot is
        offset D0 = 140 mm along world +X.

    Returns
    -------
    th_a : float  – base rotation angle (rad)
    th_b : float  – shoulder angle in motor frame (rad)
    th_c : float  – elbow angle in motor frame (rad)

    Raises
    ------
    Emits a UserWarning (but continues) when the target is outside the
    robot's reachable workspace.
    """
   

    return th_a, th_b, th_c


def ik_to_motor_deg(th_a, th_b, th_c):
    """
    Convert IK output angles to physical motor angles in degrees.

    Applies the mapping derived in Robot_HWC_Soln.m:
        th1 =  th_a
        th2 =  th_b
        th3 = -(th_c + th_b)

    Parameters
    ----------
    th_a, th_b, th_c : float or array-like
        IK joint angles in radians.

    Returns
    -------
    th1_deg, th2_deg, th3_deg : float or ndarray
        Physical motor angles in degrees (home-relative).
    """
    th_a = np.asarray(th_a)
    th_b = np.asarray(th_b)
    th_c = np.asarray(th_c)

    th1_deg = np.degrees(th_a)
    th2_deg = np.degrees(th_b)
    th3_deg = np.degrees(-(th_c + th_b))

    return th1_deg, th2_deg, th3_deg
