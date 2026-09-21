from setuptools import find_packages, setup

package_name = "wuji_controller"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="Wuji Hand2 device adapter",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wuji_hand_serials = wuji_controller.read_hand_serials:main",
        ],
    },
)
