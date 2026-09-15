from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'data_collector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eai',
    maintainer_email='1107587798@QQ.COM',
    description='Data collector for unitree robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'data_collector_node = data_collector.data_collector_node:main',
            'keyboard_controller = data_collector.keyboard_controller:main',
        ],
    },
)
