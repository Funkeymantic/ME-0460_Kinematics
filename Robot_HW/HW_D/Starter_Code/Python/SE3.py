"""
robotics_utils.SE3
===========
Operations on SE(3) -- the group of rigid body transformations.
Based on Murray, Li,  "A Mathematical Introduction to Robotic Manipulation" (1994), Chapter 2.

Twist coordinates xi in R^6 are ordered as xi = [v, w] where:
    w in R^3  -- rotational component (axis)
    v in R^3  -- translational component

    For a revolute joint (axis w through point q):  xi = [-w x q, w]  [eq. 2.22 / p.47]
    For a prismatic joint (direction v):            xi = [v, 0]        [eq. 2.28]

Functions
---------
wedge(xi)        : R^6 -> se(3)   twist coords to 4x4 matrix         [Murray eq. 2.31]
unwedge(xi_hat)  : se(3) -> R^6   inverse of wedge                   [Murray eq. 2.30]
twist(w, q)      : build revolute twist from axis w and point q      [Murray p. 47]
expt(xi, theta)  : se(3) -> SE(3) matrix exponential                 [Murray eq. 2.36]
Ad(T)            : adjoint representation of T in SE(3)              [Murray eq. 2.58]
logt(T)          : SE(3) -> (xi, theta) inverse of exp               [Murray Prop. 2.9]
inv(T)           : SE(3) -> SE(3) explicit group inverse             [Murray eq. 2.22]

Symbolic support
----------------
wedge(), unwedge(), twist(), expt(), Ad(), and inv() accept either plain Python/NumPy
numeric inputs or SymPy symbolic expressions and return the appropriate type
automatically.  logt() is numeric only.
"""

import numpy as np
import SO3


# ---------------------------------------------------------------------------
# Internal dispatch helpers (mirrors SO3._is_sym / _asarray etc.)
# ---------------------------------------------------------------------------

def _is_sym(x):
    try:
        import sympy as sp
        if isinstance(x, sp.Basic):
            return True
        try:
            return any(isinstance(e, sp.Basic) for e in np.asarray(x).ravel())
        except Exception:
            return False
    except ImportError:
        return False


def _asarray(x, sym):
    if sym:
        import sympy as sp
        return sp.Matrix(x)
    return np.asarray(x, dtype=float)


def _eye(n, sym):
    if sym:
        import sympy as sp
        return sp.eye(n)
    return np.eye(n)


def _zeros_mat(rows, cols, sym):
    if sym:
        import sympy as sp
        return sp.zeros(rows, cols)
    return np.zeros((rows, cols))


def _vstack(blocks, sym):
    """Stack a list of row-compatible matrices vertically."""
    if sym:
        import sympy as sp
        return sp.Matrix.vstack(*blocks)
    return np.vstack(blocks)


def _hstack(blocks, sym):
    """Stack a list of column-compatible matrices horizontally."""
    if sym:
        import sympy as sp
        return sp.Matrix.hstack(*blocks)
    return np.hstack(blocks)


def _cross(a, b, sym):
    """Cross product of two 3-vectors."""
    if sym:
        import sympy as sp
        a, b = sp.Matrix(a), sp.Matrix(b)
        return sp.Matrix([
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        ])
    return np.cross(np.asarray(a, dtype=float), np.asarray(b, dtype=float))


def _concat(parts, sym):
    """Concatenate a list of 1-D vectors into one."""
    if sym:
        import sympy as sp
        return sp.Matrix([e for part in parts for e in part])
    return np.concatenate([np.asarray(p, dtype=float).ravel() for p in parts])


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def wedge(xi):
    """
    Wedge (hat) map for twists. Converts 6-vector twist coords to 4x4 se(3) matrix.

    Murray eq. 2.31:
        [v, w]^wedge = [[hat(w),  v],
                        [0,       0]]

    Parameters
    ----------
    xi : array-like, shape (6,)  -- twist coordinates [v, w]; numeric or symbolic

    Returns
    -------
    xi_hat : ndarray or sp.Matrix, shape (4,4)
    """
    sym = _is_sym(xi)
    xi = _asarray(xi, sym)
    v = _asarray(xi[:3], sym)   # xi[:3] on a SymPy Matrix returns a plain list
    w = _asarray(xi[3:], sym)   # _asarray converts it to a proper sp.Matrix

    top = _hstack([SO3.hat(w), v if sym else v.reshape(3, 1)], sym)
    bot = _hstack([_zeros_mat(1, 3, sym), _zeros_mat(1, 1, sym)], sym)
    return _vstack([top, bot], sym)


def unwedge(xi_hat):
    """
    Extract 6-vector [v, w] from a 4x4 se(3) matrix.
    Inverse of wedge(), Murray eq. 2.30.

    Parameters
    ----------
    xi_hat : array-like, shape (4,4)  -- numeric or symbolic

    Returns
    -------
    xi : ndarray or sp.Matrix, shape (6,)  -- [v, w]
    """
    sym = _is_sym(xi_hat)
    xi_hat = _asarray(xi_hat, sym)
    v = xi_hat[:3, 3]
    w = SO3.vee(xi_hat[:3, :3])
    return _concat([v, w], sym)


# Keep vee as an alias so existing code doesn't break
def vee(xi_hat):
    return unwedge(xi_hat)


def twist(w, q):
    """
    Construct a revolute twist from a rotation axis and a point on the axis.

    Murray (p. 47, pure rotation case of screw):
        xi = [-w x q, w]    (with ||w|| = 1)

    Parameters
    ----------
    w : array-like, shape (3,)  -- unit rotation axis; numeric or symbolic
    q : array-like, shape (3,)  -- any point on the axis; numeric or symbolic

    Returns
    -------
    xi : ndarray or sp.Matrix, shape (6,)  -- twist coordinates [v, w]
    """
    sym = _is_sym(w) or _is_sym(q)
    w = _asarray(w, sym)
    q = _asarray(q, sym)
    v = -_cross(w, q, sym)
    return _concat([v, w], sym)


def expt(xi, theta):
    """
    Matrix exponential on SE(3).
    Computes T = exp(wedge(xi) * theta).

    Murray eq. 2.36 (w != 0, revolute case):
        T = [[ exp(hat(w)*theta),   (I - exp(hat(w)*theta))(w x v) + w*w^T*v*theta ],
             [ 0,                   1                                               ]]

    Murray eq. 2.32 (w = 0, prismatic case):
        T = [[ I,   v*theta ],
             [ 0,   1       ]]

    Parameters
    ----------
    xi    : array-like, shape (6,)  -- twist coordinates [v, w]; numeric or symbolic
    theta : float or sp.Expr        -- magnitude of motion

    Returns
    -------
    T : ndarray or sp.Matrix, shape (4,4)

    Notes
    -----
    For numeric inputs ||w|| is checked and w is normalized if non-zero.
    For symbolic inputs the revolute branch is always taken — pass xi with
    ||w|| = 1 as in the standard Murray convention.
    """
    sym = _is_sym(xi) or _is_sym(theta)
    xi = _asarray(xi, sym)
    v, w = xi[:3], xi[3:]

    I3 = _eye(3, sym)
    I4 = _eye(4, sym)

    if sym:
        import sympy as sp
        # Revolute branch (symbolic — caller ensures ||w|| = 1)
        R = SO3.expr(w, theta)
        w_col = sp.Matrix(w)
        v_col = sp.Matrix(v)
        p = (I3 - R) * SO3.hat(w) * v_col + w_col * (w_col.dot(v_col)) * theta
        T = sp.eye(4)
        T[:3, :3] = R
        T[:3,  3] = p
        return T

    # Numeric path
    if np.isclose(np.linalg.norm(w), 0):
        # Prismatic -- eq. 2.32
        T = np.eye(4)
        T[:3, 3] = v * theta
        return T

    # Revolute -- eq. 2.36
    w = w / np.linalg.norm(w)
    R = SO3.expr(w, theta)
    p = (np.eye(3) - R) @ np.cross(w, v) + w * (w @ v) * theta
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = p
    return T


def Ad(T):
    """
    Adjoint representation of a transformation T in SE(3).
    Maps body twists to spatial twists: V^s = Ad(g) * V^b

    Murray eq. 2.58:
        Ad(g) = [[ R,         hat(p) @ R ],
                 [ 0_{3x3},   R          ]]

    Parameters
    ----------
    T : array-like, shape (4,4)  -- numeric or symbolic

    Returns
    -------
    AdT : ndarray or sp.Matrix, shape (6,6)
    """
    sym = _is_sym(T)
    T = _asarray(T, sym)
    R = T[:3, :3]
    p = T[:3,  3]

    pR = SO3.hat(p) * R if sym else SO3.hat(p) @ np.asarray(R)
    z  = _zeros_mat(3, 3, sym)

    top = _hstack([R,  pR], sym)
    bot = _hstack([z,  R ], sym)
    return _vstack([top, bot], sym)


def logt(T):
    """
    Matrix logarithm on SE(3).  **Numeric only.**
    Returns twist xi and scalar theta such that T = exp(xi, theta).

    Derived constructively in proof of Proposition 2.9 (p. 43):

        Step 1: Find w, theta from SO3.logr(R)          [Murray Prop. 2.5]
        Step 2: Solve A*v = p for v, where              [Murray eq. 2.38]
                A = (I - exp(hat(w)*theta)) * hat(w) + w*w^T*theta

    Parameters
    ----------
    T : array-like, shape (4,4)  -- homogeneous transformation in SE(3)

    Returns
    -------
    xi    : ndarray, shape (6,)  -- twist coordinates [v, w]
    theta : float                -- magnitude of motion
    """
    T = np.asarray(T, dtype=float)
    R, p = T[:3, :3], T[:3, 3]

    w, theta = SO3.logr(R)

    if np.isclose(theta, 0):
        # Pure translation (Murray Prop. 2.9 Case 1)
        p_norm = np.linalg.norm(p)
        xi = np.concatenate([p / p_norm if not np.isclose(p_norm, 0) else p,
                              np.zeros(3)])
        return xi, float(p_norm)

    # Revolute case -- solve eq. 2.38 for v
    A = (np.eye(3) - R) @ SO3.hat(w) + np.outer(w, w) * theta
    v = np.linalg.solve(A, p)

    return np.concatenate([v, w]), theta


def inv(T):
    """
    Explicit group inverse in SE(3).

    For T = [[R, p], [0, 1]], the inverse is:
        T^{-1} = [[ R^T,  -R^T p ],
                  [   0,       1  ]]

    This exploits the SE(3) structure and is more efficient and
    numerically stable than a generic matrix inverse.

    Parameters
    ----------
    T : array-like, shape (4,4)  -- numeric or symbolic

    Returns
    -------
    T_inv : ndarray or sp.Matrix, shape (4,4)
    """
    sym = _is_sym(T)
    T = _asarray(T, sym)
    R = T[:3, :3]
    p = T[:3,  3]

    if sym:
        import sympy as sp
        RT = R.T
        T_inv = sp.eye(4)
        T_inv[:3, :3] = RT
        T_inv[:3,  3] = -RT * sp.Matrix(p)
        return T_inv

    RT = R.T
    T_inv = np.eye(4)
    T_inv[:3, :3] = RT
    T_inv[:3,  3] = -RT @ p
    return T_inv
