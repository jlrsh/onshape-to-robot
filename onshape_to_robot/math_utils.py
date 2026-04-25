"""
Small pure math helpers. Kept here to avoid duplication across modules.
"""
from __future__ import annotations

import numpy as np


def normalize_angle_pi(angle: float) -> float:
    """
    Normalize an angle to the half-open interval (-pi, pi]. Needed because the
    Onshape API can report accumulated multi-revolution rotation values while
    joint bounds only care about the principal angle.
    """
    return float(np.arctan2(np.sin(angle), np.cos(angle)))
