"""Small control helpers shared by the integrated lane controller."""

from bisect import bisect_right
from typing import Sequence

from geometry_msgs.msg import Point


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def interpolate_clamped(
    value: float,
    inputs: Sequence[float],
    outputs: Sequence[float],
) -> float:
    if len(inputs) != len(outputs) or len(inputs) < 2:
        raise ValueError("lookup inputs and outputs must have the same length >= 2")
    if any(right <= left for left, right in zip(inputs, inputs[1:])):
        raise ValueError("lookup inputs must be strictly increasing")
    if value <= inputs[0]:
        return float(outputs[0])
    if value >= inputs[-1]:
        return float(outputs[-1])
    upper = bisect_right(inputs, value)
    lower = upper - 1
    ratio = (value - inputs[lower]) / (inputs[upper] - inputs[lower])
    return float(outputs[lower] + ratio * (outputs[upper] - outputs[lower]))


def inverse_lookup_table(
    inputs: Sequence[float],
    outputs: Sequence[float],
) -> tuple[list[float], list[float]]:
    if len(inputs) != len(outputs) or len(inputs) < 2:
        raise ValueError("lookup inputs and outputs must have the same length >= 2")
    inverse_pairs = sorted(zip(outputs, inputs))
    inverse_inputs = [float(value) for value, _ in inverse_pairs]
    inverse_outputs = [float(value) for _, value in inverse_pairs]
    interpolate_clamped(0.0, inverse_inputs, inverse_outputs)
    return inverse_inputs, inverse_outputs


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point
