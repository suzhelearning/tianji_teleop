"""Build numerical kernels; all package metadata lives in pyproject.toml."""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

setup(
    ext_modules=[
        Pybind11Extension(
            "retargeting.wuji_retargeting._native",
            ["retargeting/wuji_retargeting/native.cpp"],
            cxx_std=17,
            extra_compile_args=["-O3", "-ffp-contract=off"],
        ),
    ],
    cmdclass={"build_ext": build_ext},
)
