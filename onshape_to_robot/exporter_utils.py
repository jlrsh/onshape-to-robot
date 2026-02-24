import math
import numpy as np
from xml.sax.saxutils import escape


def xml_escape(unescaped: str) -> str:
    """
    Escapes XML characters in a string so that it can be safely added to an XML file
    """
    return escape(unescaped, entities={"'": "&apos;", '"': "&quot;"})


# Cyclic axis permutation: Z out/Y up/X right -> X out/Z up/Y right
# Maps: old X->new Y, old Y->new Z, old Z->new X
T_x_forward = np.array([
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
], dtype=float)


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
