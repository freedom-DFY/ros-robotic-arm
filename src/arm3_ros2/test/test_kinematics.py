import math
import random

import pytest

from arm3_ros2.kinematics import Calibration, RADIANS_PER_TICK, WristKinematics


def model():
    calibration = Calibration([2048] * 3, [1] * 3, [0] * 3, [4095] * 3)
    return calibration, WristKinematics(0.24606, 0.24225, 0.0,
                                       [math.pi / 2, -1.444, -1.444], calibration)


def test_fk_ik_roundtrip():
    _, kin = model()
    rng = random.Random(43)
    for _ in range(100):
        q = tuple(rng.uniform(-2.5, 2.5) for _ in range(3))
        point = kin.forward(q)
        recovered = kin.inverse(point, q)
        assert math.dist(kin.forward(recovered), point) < 1e-8
        assert math.dist(recovered, q) < 1e-8


def test_calibration_direction_limits_and_quantization():
    cal = Calibration([2048] * 3, [-1, 1, -1], [100, 200, 300], [3900, 3800, 3700])
    assert cal.ticks(cal.radians((2000, 2200, 1800))) == (2000, 2200, 1800)
    assert cal.radians((2047, 2048, 2049)) == (RADIANS_PER_TICK, 0.0, -RADIANS_PER_TICK)
    with pytest.raises(ValueError):
        cal.ticks((4.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        cal.ticks((float('nan'), 0.0, 0.0))


def test_straight_path_stays_on_line():
    cal, kin = model()
    start = cal.radians((2048, 2160, 1910))
    origin, target = kin.forward(start), (0.01, 0.30, 0.15)
    path = kin.line_path(start, target, 0.002, 0.30)
    assert path
    previous = start
    for index, (q, segment, jump) in enumerate(path, 1):
        fraction = index / len(path)
        expected = tuple(a + (b - a) * fraction for a, b in zip(origin, target))
        assert math.dist(kin.forward(q), expected) < 1e-8
        assert 0 < segment <= 0.002
        assert max(abs(a - b) for a, b in zip(q, previous)) <= 0.30
        assert jump <= 0.30
        cal.ticks(q)
        previous = q


def test_unreachable_or_discontinuous_path_is_rejected():
    cal, kin = model()
    start = cal.radians((2048, 2160, 1910))
    with pytest.raises(ValueError, match='workspace'):
        kin.line_path(start, (2.0, 0.0, 0.0), 0.005, 0.30)
    with pytest.raises(ValueError, match='singularity or joint limit'):
        kin.line_path(start, (0.01, 0.30, 0.15), 0.005, 1e-9)
