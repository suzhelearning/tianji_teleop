from setuptools import find_packages, setup

package_name = "mocap_policy_runtime"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    package_data={package_name: ["configs/*.yaml", "native/*.cpp"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    extras_require={
        "mocap-policy": [
            "torch==2.10.0",
            "eclipse-zenoh>=1.10,<2",
            "Pillow>=10,<13",
        ],
    },
    zip_safe=True,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="Mocap trajectory replay and reference-conditioned policy inference",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mocap_policy_runtime = mocap_policy_runtime.__main__:main",
        ],
    },
)
