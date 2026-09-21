from setuptools import find_packages, setup

package_name = "tianji_tools"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="Operator command surface and personnel-profile management",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tianji = tianji.cli:main",
        ],
    },
)
