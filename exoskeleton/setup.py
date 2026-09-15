"""Build the exoskeleton's per-frame numerical kernels."""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

setup(
    ext_modules=[
        Pybind11Extension(
            "data_glove_wuji_teleop._native",
            [
                "native/module.cpp",
                "native/morphology.cpp",
                "native/encoder_kinematics.cpp",
            ],
            cxx_std=17,
            extra_compile_args=["-O3", "-ffp-contract=off"],
        ),
    ],
    cmdclass={"build_ext": build_ext},
)
