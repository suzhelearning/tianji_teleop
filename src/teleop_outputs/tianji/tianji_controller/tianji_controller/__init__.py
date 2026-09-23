"""Native controller profile resources and the TJRC command-frame codec.

The package shell installs the reviewed controller profiles (under ``native/``)
and this Python codec. ``protocol`` decodes the loopback-only final-joint frame
that the native DLS worker emits; the PICO bare-hand -> SPD route uses it to
feed the same targets into its display and validation. The hardware executor,
operator terminals, safety gates and ROS observer are gone with the real-robot
routes.
"""
