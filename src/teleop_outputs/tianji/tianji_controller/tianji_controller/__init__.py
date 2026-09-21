"""Tianji arm executor and site configuration.

Owns the 200 Hz control loop, the MotionGate/StagedMotionGate authorization
state machine, the MuJoCo monitor, the native QP/IK controller and the
read-before-enable Marvin arm I/O. Hand I/O lives in ``wuji_controller``.
"""
