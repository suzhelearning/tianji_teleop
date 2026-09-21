"""Workspace-wide numeric constants shared by executor, collector and tools.

They are duplicated nowhere else: a change here changes both the acquisition
target and the schema-v1 validation bound in one place.
"""

# Standard schema-v1 acquisition target. `config/collect_real.json` must agree.
STATE_RATE_HZ = 120

# Measured-state vector layout.
ARMS_COUNT = 14
HAND_COUNT = 20
STATE_DIM = ARMS_COUNT + 2 * HAND_COUNT  # 54

# Device names used on the wire, in config and in the HDF5 layout.
DEVICES = ("arms", "left_hand", "right_hand")

# RGB geometry of every schema-v1 camera stream.
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
RGB_SHAPE = (IMAGE_HEIGHT, IMAGE_WIDTH, 3)
CAMERA_FPS = 30
CAMERA_ROLES = ("top", "left_wrist", "right_wrist")
