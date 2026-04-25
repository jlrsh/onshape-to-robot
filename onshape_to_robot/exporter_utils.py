import math
import numpy as np
from xml.sax.saxutils import escape


def xml_escape(unescaped: str) -> str:
    """
    Escapes XML characters in a string so that it can be safely added to an XML file
    """
    return escape(unescaped, entities={"'": "&apos;", '"': "&quot;"})


# Frames whose name starts with this prefix are emitted as-is by the frame
# rewriters — they represent kinematic-loop closing connections and must not be
# reoriented by frame_x_forward.
CLOSING_FRAME_PREFIX = "closing_"

# Cyclic axis permutation: Z out/Y up/X right -> X out/Z up/Y right
# Maps: old X->new Y, old Y->new Z, old Z->new X
T_x_forward = np.array([
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
], dtype=float)


def apply_frame_x_forward(T_world_frame: np.ndarray, frame_name: str, config) -> np.ndarray:
    """
    Return T_world_frame post-multiplied by T_x_forward when the config's
    frame_x_forward flag is set, except for closing-loop frames (which carry
    kinematic meaning and must not be reoriented). Otherwise returns the input
    transform unchanged.
    """
    if (
        config is not None
        and getattr(config, "frame_x_forward", False)
        and not frame_name.startswith(CLOSING_FRAME_PREFIX)
    ):
        return T_world_frame @ T_x_forward
    return T_world_frame


def rotation_matrix_to_rpy(R):
    """
    Converts a rotation matrix to rpy Euler angles
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
