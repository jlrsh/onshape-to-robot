import numpy as np

from onshape_to_robot.exporter_utils import (
    CLOSING_FRAME_PREFIX,
    T_x_forward,
    apply_frame_x_forward,
)


class _FakeConfig:
    def __init__(self, frame_x_forward: bool):
        self.frame_x_forward = frame_x_forward


class TestApplyFrameXForward:
    def test_flag_off_returns_input_unchanged(self):
        T = np.random.RandomState(0).rand(4, 4)
        out = apply_frame_x_forward(T, "some_frame", _FakeConfig(False))
        assert np.array_equal(out, T)

    def test_flag_on_applies_permutation(self):
        T = np.eye(4)
        out = apply_frame_x_forward(T, "base_link", _FakeConfig(True))
        assert np.allclose(out, T_x_forward)

    def test_closing_frame_skipped_even_with_flag(self):
        T = np.eye(4)
        out = apply_frame_x_forward(T, f"{CLOSING_FRAME_PREFIX}loop0", _FakeConfig(True))
        assert np.array_equal(out, T)

    def test_none_config_returns_input_unchanged(self):
        T = np.eye(4)
        out = apply_frame_x_forward(T, "frame", None)
        assert np.array_equal(out, T)
