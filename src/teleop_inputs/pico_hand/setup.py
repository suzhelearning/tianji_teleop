from setuptools import setup

package_name = "pico2_hands"

# Configuration remains package data; native artifacts are installed separately
# into install/spd by build_native.sh.
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
            # The shared display and arm DLS models come from the main
            # workspace; this package owns only its runtime configuration.
            "config/*.yaml",
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
    description="PICO2 shared-root DLS and native Hand2 bare-hand simulation (no hardware)",
    license="Apache-2.0",
)
