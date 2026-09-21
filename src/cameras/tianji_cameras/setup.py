from setuptools import find_packages, setup

package_name = "tianji_cameras"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/cameras.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tianji operators",
    maintainer_email="ops@example.com",
    description="RealSense camera launch, readiness monitor, preview and PICO video",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tianji_camera_preflight = tianji_cameras.preflight:main",
            "tianji_camera_monitor = tianji_cameras.monitor:main",
            "tianji_camera_preview = tianji_cameras.preview:main",
            "tianji_pico_camera = tianji_cameras.pico_stream:main",
        ],
    },
)
