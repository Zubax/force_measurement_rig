from typing import Generic, TypeVar
import numpy as np
from numpy.typing import NDArray
from serial_interface import ForceSensorReading

T = TypeVar("T")


class MovingAverage(Generic[T]):
    """
    >>> ma = MovingAverage(3, 0)
    >>> ma(1)  # doctest: +ELLIPSIS
    0.333...
    >>> ma(2)  # doctest: +ELLIPSIS
    1.0
    >>> ma(3)  # doctest: +ELLIPSIS
    2.0

    >>> maa = MovingAverage(3, np.array([1, 2, 3]))
    >>> maa(np.array([2, 2, 2]))  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    array([1.333..., 2..., 2.666...])
    >>> maa(np.array([2, 2, 2]))  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    array([1.666..., 2..., 2.333...])
    >>> maa(np.array([2, 2, 2]))  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    array([2..., 2..., 2...])
    """

    def __init__(self, depth: int, initial: T) -> None:
        self._values = [initial] * depth
        self._sum = sum(self._values)  # type: ignore
        self._index = 0

    def __call__(self, value: T) -> T:
        self._sum -= self._values[self._index]  # type: ignore
        self._values[self._index] = value
        self._sum += value  # type: ignore
        self._index = (self._index + 1) % len(self._values)
        return self._sum * (1 / len(self._values))  # type: ignore


def compute_forces(rd: ForceSensorReading) -> NDArray[np.float64]:
    return np.array(
        [
            np.polynomial.Polynomial(np.flip(coe.flatten()))(float(x))
            for x, coe in zip(rd.adc_readings, rd.calibration.T)
        ]
    )
