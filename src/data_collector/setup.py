from setuptools import find_packages, setup

package_name = "data_collector"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    package_data={package_name: ["web/visualize.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="Schema-v1 episode collector over ROS feedback and camera topics",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "data_collector_node = data_collector.node:main",
        ],
    },
)
