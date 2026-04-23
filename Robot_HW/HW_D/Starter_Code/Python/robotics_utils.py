"""
robotics_utils
============
Robotics utilities based on Murray's
"A Mathematical Introduction to Robotic Manipulation"

Usage
-----
import robotics_utils as ru

ru.SO3.hat(w)
ru.SO3.expr(w, theta)
ru.SE3.twist(w, q)
ru.SE3.expt(xi, theta)
ru.SE3.inv(T)

ru.lambdify(expr, syms)          # SymPy expr -> fast callable
ru.export_func(expr, syms, ...)  # SymPy expr -> .py file + callable
"""

import SO3
import SE3


# ---------------------------------------------------------------------------
# Lambdify utilities
# ---------------------------------------------------------------------------

def lambdify(expr, syms, backend='numpy'):
    """
    Convert a SymPy expression (or Matrix) to a fast numerical callable.
    Thin wrapper around sp.lambdify — option 1 pattern from the demo notebook.

    Parameters
    ----------
    expr    : SymPy expression or Matrix
    syms    : tuple/list of SymPy symbols  -- function arguments, in order
    backend : str, default 'numpy'         -- 'numpy', 'scipy', or 'math'

    Returns
    -------
    f : callable
        Numerical function with the same argument order as `syms`.

    Example
    -------
    th = sp.Symbol('th')
    f  = ru.lambdify(sp.cos(th), (th,))
    f(np.pi)   # -> -1.0
    """
    import sympy as sp
    return sp.lambdify(syms, expr, backend)


def export_func(expr, syms, func_name, filepath, backend='numpy'):
    """
    Export a SymPy expression as a named Python function in a .py file,
    then return it as a callable.  Option 2 pattern from the demo notebook.

    Writes ``filepath`` (adds .py if missing) containing::

        import numpy

        def func_name(sym0, sym1, ...):
            return <lambdified expression>

    Parameters
    ----------
    expr      : SymPy expression or Matrix
    syms      : tuple/list of SymPy symbols  -- argument order for the function
    func_name : str   -- name of the Python function to write
    filepath  : str   -- output file path (e.g. 'gab_numerical' or 'gab_numerical.py')
    backend   : str, default 'numpy'

    Returns
    -------
    f : callable
        The same function returned directly (no manual import required).

    Example
    -------
    f = ru.export_func(gab_th, (d0, d1, d2, d3, th),
                       func_name='get_gab', filepath='gab_numerical')
    f(1, 2, 3, 4, np.radians(45))
    """
    from sympy.utilities.lambdify import lambdastr
    from sympy.printing.lambdarepr import NumPyPrinter
    import sympy as sp

    if not filepath.endswith('.py'):
        filepath = filepath + '.py'

    sym_names = [str(s) for s in syms]

    kw = {'printer': NumPyPrinter} if backend == 'numpy' else {}
    code_body = lambdastr(syms, expr, **kw)

    with open(filepath, 'w') as f:
        f.write('import numpy\n\n')
        f.write(f'def {func_name}({", ".join(sym_names)}):\n')
        f.write(f'    return ({code_body})({", ".join(sym_names)})\n')

    # Return the callable directly — no import dance needed by the caller
    return sp.lambdify(syms, expr, backend)
