"""ament_python package: retargeting kernels, example commands and model assets."""
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import find_packages, setup

package_name = "wuji_retargeting"

setup(
    name=package_name,
    version="0.3.0",
    # `example` is a top-level package beside the module directory; it ships the
    # runnable commands and their assets.
    packages=find_packages(exclude=["test", "tests"]),
    # Every example resolves its assets relative to its own module file, so the
    # hand model tree and the example data must be installed with the packages.
    package_data={
        package_name: ["wuji-description/**/*"],
        "example": [
            "config/*.yaml",
            "config/**/*.yaml",
            "data/*",
            "data/**/*",
            "utils/mujoco-sim/**/*",
        ],
    },
    ext_modules=[
        Pybind11Extension(
            "wuji_retargeting._native",
            ["wuji_retargeting/native.cpp"],
            cxx_std=17,
            extra_compile_args=["-O3", "-ffp-contract=off"],
        ),
    ],
    cmdclass={"build_ext": build_ext},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="Wuji hand retargeting: algorithms, input adapters and hand model assets.",
    license="MIT",
)
