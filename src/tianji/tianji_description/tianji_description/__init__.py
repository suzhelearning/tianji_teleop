"""Shared description resources and their small pure-Python helpers.

``home_config`` resolves the deployment Home vector and controller initial
posture; ``model_assets`` adds the display-only object mesh to a MuJoCo model.
Both are imported by the native controller tests, the simulator and the
collector metadata builder, so they live here rather than in any one consumer.
"""
