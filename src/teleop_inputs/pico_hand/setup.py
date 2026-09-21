from setuptools import setup

package_name = "pico2_hands"

# Standard ament_python layout: the importable package lives in python/ so a
# colcon symlink install can mirror it. `config/` and `native/` sit inside the
# package because `run_sim.py`, the offline scripts and the native launcher
# address them through `Path(__file__).parent`, which must hold in the
# installed tree as well as in the checkout.
setup(
    name=package_name,
    version="0.3.0",
    package_dir={"": "python"},
    packages=[
        package_name,
        package_name + ".reference",
        package_name + ".scripts",
        package_name + ".tests",
    ],
    package_data={
        package_name: [
            # Simulation configuration and the V131 kinematics models this
            # package owns (the shared display model comes from
            # tianji_description at runtime).
            "config/*.yaml",
            "native/models/*",
            # Native artifacts built by `native/CMakeLists.txt` and
            # `native/hand/optimizer/CMakeLists.txt` into their own build trees.
            "native/build/pico2-v131/pico2_v131_worker",
            "native/build/pico2-hand/tianji_hand_native_worker",
            "native/build/pico2-hand/libtianji_hand_optimizer.so",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="PICO2 bare-hand simulation with V131 or shared-root DLS (no hardware)",
    license="Apache-2.0",
)
