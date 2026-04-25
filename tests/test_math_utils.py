import math

import pytest

from onshape_to_robot.math_utils import normalize_angle_pi


class TestNormalizeAnglePi:
    @pytest.mark.parametrize(
        "raw,expected",
        [
            (0.0, 0.0),
            (math.pi, math.pi),
            (2 * math.pi, 0.0),
            (-2 * math.pi, 0.0),
            (math.pi / 4, math.pi / 4),
        ],
    )
    def test_wrap(self, raw, expected):
        assert math.isclose(normalize_angle_pi(raw), expected, abs_tol=1e-9)

    def test_odd_multiples_of_pi_land_on_plus_or_minus_pi(self):
        # `arctan2(0, -1)` has ambiguous sign in IEEE-754 when the sine
        # accumulates tiny rounding error — both -π and +π are valid
        # representations of the same angle.
        for raw in (3 * math.pi, -3 * math.pi):
            out = normalize_angle_pi(raw)
            assert math.isclose(abs(out), math.pi, abs_tol=1e-9)

    def test_large_positive(self):
        # 10.5 * pi -> principal part 0.5 * pi
        assert math.isclose(normalize_angle_pi(10.5 * math.pi), 0.5 * math.pi, abs_tol=1e-9)

    def test_large_negative(self):
        assert math.isclose(normalize_angle_pi(-10.5 * math.pi), -0.5 * math.pi, abs_tol=1e-9)
