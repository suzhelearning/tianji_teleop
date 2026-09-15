#!/usr/bin/env bash
# Source after changing to the project root; component overrides remain explicit.
TIANJI_PYTHON="${TIANJI_PYTHON:-$PWD/.venv/bin/python}"
if [[ ! -x "$TIANJI_PYTHON" ]]; then
  printf '%s\n' \
    "Project Python missing: $TIANJI_PYTHON; no hardware was contacted." \
    'Create .venv with your ROS-compatible Python, then install: python -m pip install -e ".[collection,retargeting]"' \
    'Or set TIANJI_PYTHON to an explicitly provisioned Python environment.' >&2
  exit 1
fi
export TIANJI_PYTHON
