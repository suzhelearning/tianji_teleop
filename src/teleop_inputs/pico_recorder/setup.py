from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pico_recorder'

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
    description='PICO HDF5 and MP4 recorder',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pico_recorder_node = pico_recorder.pico_recorder_node:main',
            'pico_recorder_keyboard = pico_recorder.pico_recorder_node:keyboard_main',
        ],
    },
)
