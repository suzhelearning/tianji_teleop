"""Tianji arm executor and site configuration.

Owns the 200 Hz control loop, the MotionGate/StagedMotionGate authorization
state machine, the MuJoCo monitor, the native DLS/Ruckig controller and the
read-before-enable Marvin arm I/O. Hand I/O lives in ``wuji_controller``.
"""
