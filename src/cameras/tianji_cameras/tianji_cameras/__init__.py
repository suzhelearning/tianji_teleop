"""RealSense camera launch, readiness monitor and preview.

The official ``realsense2_camera`` driver is the only owner of every RGB
pipeline: nothing here opens a RealSense pipeline or stops someone else's
driver. Depends on ``tianji_runtime`` for the shared resource and stream
contract.
"""
