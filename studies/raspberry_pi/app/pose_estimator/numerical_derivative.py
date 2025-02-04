# pylint: disable=C0103,C0114,C0116,E0611,R0913
# mypy: disable-error-code="import-untyped"
# see numericalDerivative.h

# pybind wants to wrap concrete types, which would have been
# a whole lot of them, so i reimplemented the part of this that
# i needed, using the python approach to "generic" typing.

from typing import Callable, TypeVar
import numpy as np

Y = TypeVar("Y")
X = TypeVar("X")
X1 = TypeVar("X1")
X2 = TypeVar("X2")
X3 = TypeVar("X3")
X4 = TypeVar("X4")
X5 = TypeVar("X5")
X6 = TypeVar("X6")


def local(a: Y, b: Y) -> np.ndarray:
    if type(a) is not type(b):
        raise TypeError(f"a {type(a)} b {type(b)}")
    if isinstance(a, np.ndarray):
        return b - a
    if isinstance(a, (float, int)):
        return np.array([[b - a]])  # type:ignore
    # there is no common superclass for Y
    return a.localCoordinates(b)  # type:ignore


def retract(a, b):
    if isinstance(a, (np.ndarray, float, int)):
        return a + b
    return a.retract(b)


def numericalDerivative11(h: Callable[[X], Y], x: X, delta=1e-5) -> np.ndarray:
    hx: Y = h(x)
    zeroY = local(hx, hx)
    m = zeroY.shape[0]
    zeroX = local(x, x)
    N = zeroX.shape[0]
    dx = np.zeros(N)
    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = local(hx, h(retract(x, dx)))  # type:ignore
        dx[j] = -delta
        dy2 = local(hx, h(retract(x, dx)))  # type:ignore
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21(
    h: Callable[[X1, X2], Y], x1: X1, x2: X2, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2), x1, delta)


def numericalDerivative22(
    h: Callable[[X1, X2], Y], x1: X1, x2: X2, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x), x2, delta)


def numericalDerivative31(
    h: Callable[[X1, X2, X3], Y], x1: X1, x2: X2, x3: X3, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2, x3), x1, delta)


def numericalDerivative32(
    h: Callable[[X1, X2, X3], Y], x1: X1, x2: X2, x3: X3, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x, x3), x2, delta)


def numericalDerivative33(
    h: Callable[[X1, X2, X3], Y], x1: X1, x2: X2, x3: X3, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x), x3, delta)


def numericalDerivative61(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2, x3, x4, x5, x6), x1, delta)
