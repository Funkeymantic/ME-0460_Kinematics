"""
robotics_utils.SO3
===========
Operations on SO(3) — the group of rotation matrices.
Based on Murray, Li "A Mathematical Introduction to Robotic Manipulation" (1994), Chapter 2.

Functions
---------
hat(w)       : R^3 -> so(3)  skew-symmetric (hat) map        [Murray eq. 2.4]
vee(W)       : so(3) -> R^3  inverse of hat
expr(w, th)  : so(3) -> SO(3) via Rodrigues' formula         [Murray eq. 2.14]
logr(R)      : SO(3) -> (w, theta)  inverse of exp           [Murray Prop. 2.5, eq. 2.17-2.18]

Symbolic support
----------------
hat(), vee(), and expr() accept either plain Python/NumPy numeric inputs or
SymPy symbolic expressions and return the appropriate type automatically.
logr() is numeric only (it calls arccos/arcsin on matrix entries).
"""

import numpy as np


# ---------------------------------------------------------------------------
# Internal dispatch helpers — not part of the public API
# ---------------------------------------------------------------------------

def _is_sym(x):
    """Return True if x contains any SymPy objects."""
    try:
        import sympy as sp
        if isinstance(x, sp.Basic):
            return True
        # Check elements of a list/array/Matrix
        try:
            return any(isinstance(e, sp.Basic) for e in np.asarray(x).ravel())
        except Exception:
            return False
    except ImportError:
        return False


def _eye(n, sym):
    if sym:
        import sympy as sp
        return sp.eye(n)
    return np.eye(n)


def _zeros(shape, sym):
    if sym:
        import sympy as sp
        return sp.zeros(*shape)
    return np.zeros(shape)


def _asarray(x, sym):
    if sym:
        import sympy as sp
        return sp.Matrix(x)
    return np.asarray(x, dtype=float)


def _sin(x, sym):
    return __import__('sympy').sin(x) if sym else np.sin(x)


def _cos(x, sym):
    return __import__('sympy').cos(x) if sym else np.cos(x)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def hat(w):
    """
    Skew-symmetric matrix (hat map) for a 3-vector w.
    Maps w in R^3 to its skew-symmetric matrix W in so(3).

    Murray eq. 2.4:
        hat(w) = [[ 0,  -w3,  w2],
                  [ w3,  0,  -w1],
                  [-w2,  w1,  0 ]]

    Parameters
    ----------
    w : array-like, shape (3,)  -- numeric or symbolic

    Returns
    -------
    w_hat : ndarray or sp.Matrix, shape (3,3)
    """
    sym = _is_sym(w)
    w = _asarray(w, sym)
    if sym:
        import sympy as sp
        return sp.Matrix([
            [    0, -w[2],  w[1]],
            [ w[2],     0, -w[0]],
            [-w[1],  w[0],     0]
        ])
    return np.array([
        [ 0,    -w[2],  w[1]],
        [ w[2],  0,    -w[0]],
        [-w[1],  w[0],  0   ]
    ])


def vee(w_hat):
    """
    Inverse hat map. Extracts the 3-vector from a skew-symmetric matrix.
    Inverse of hat(), Murray eq. 2.4.

    Parameters
    ----------
    w_hat : array-like, shape (3,3)  -- numeric or symbolic

    Returns
    -------
    w : ndarray or sp.Matrix, shape (3,)
    """
    sym = _is_sym(w_hat)
    w_hat = _asarray(w_hat, sym)
    if sym:
        import sympy as sp
        return sp.Matrix([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])
    return np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])


def expr(w, theta):
    """
    Matrix exponential on SO(3) -- Rodrigues' formula.
    Computes R = exp(hat(w) * theta) for unit rotation axis w.

    Murray eq. 2.14:
        R = I + sin(theta)*hat(w) + (1 - cos(theta))*hat(w)^2

    Parameters
    ----------
    w     : array-like, shape (3,)  -- rotation axis (unit vector); numeric or symbolic
    theta : float or sp.Expr        -- rotation angle in radians

    Returns
    -------
    R : ndarray or sp.Matrix, shape (3,3)

    Notes
    -----
    For numeric inputs w is normalized internally.
    For symbolic inputs normalization is skipped — the caller is responsible
    for passing a unit-vector expression (as in the MATLAB workflow).
    """
    sym = _is_sym(w) or _is_sym(theta)
    w = _asarray(w, sym)

    if not sym:
        w = w / np.linalg.norm(w)   # normalize numeric axis

    w_hat = hat(w)
    I = _eye(3, sym)
    s = _sin(theta, sym)
    c = _cos(theta, sym)

    if sym:
        return I + s * w_hat + (1 - c) * (w_hat * w_hat)
    return I + w_hat * s + (w_hat @ w_hat) * (1 - c)


def logr(R):
    """
    Matrix logarithm on SO(3).  **Numeric only.**
    Returns axis w (unit vector) and angle theta such that R = exp(w, theta).

    NOT a single equation in Murray -- it is the constructive inverse
    derived in the proof of Proposition 2.5 (pp. 29-30):

        theta = arccos( (trace(R) - 1) / 2 )                    [eq. 2.17]
        w     = 1/(2 sin(theta)) * [r32-r23, r13-r31, r21-r12]  [eq. 2.18]

    Parameters
    ----------
    R : array-like, shape (3,3)  -- rotation matrix in SO(3)

    Returns
    -------
    w     : ndarray, shape (3,)  -- unit rotation axis
    theta : float                -- rotation angle in radians, in [0, pi]

    Special cases (Murray Prop. 2.5)
    ----------------------------------
    theta = 0  : R is identity; axis undefined, returns (zeros, 0)
    theta = pi : sin(theta)=0 so eq. 2.18 breaks; axis found from diagonal of R
    """
    R = np.asarray(R, dtype=float)

    # eq. 2.17
    cos_theta = np.clip((np.trace(R) - 1) / 2, -1, 1)
    theta = np.arccos(cos_theta)

    if np.isclose(theta, 0):
        return np.zeros(3), 0.0

    if np.isclose(theta, np.pi):
        # eq. 2.18 has sin(theta)=0 in denominator -- use column of R instead.
        # Pick largest diagonal entry for numerical stability.
        i = np.argmax(np.diag(R))
        w = R[:, i] + np.eye(3)[i]
        return w / np.linalg.norm(w), float(theta)

    # eq. 2.18
    w = np.array([R[2,1] - R[1,2],
                  R[0,2] - R[2,0],
                  R[1,0] - R[0,1]]) / (2 * np.sin(theta))
    return w, theta
