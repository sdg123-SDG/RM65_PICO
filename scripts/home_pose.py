"""Default RM65 initial pose.

Recorded from the real arm on 2026-10-10 after drag teach, once the gripper
had been placed by hand. Joint angles are the controller's degrees.
"""

import math

HOME_JOINTS_DEG = (-5.5, -1.8, -85.3, -1.5, -77.6, -2.0)
HOME_XYZ_M = (0.254, -0.028, 0.368)
HOME_JOINTS_RAD = tuple(math.radians(d) for d in HOME_JOINTS_DEG)
