# setup.py: ament_python build entry for fr3_calibration.
# fr3_calibration: hand-eye calibration package wiring realsense2_camera +
# aruco_ros + easy_handeye2 with FR3-specific frames.

from glob import glob
import os
from setuptools import setup

package_name = 'fr3_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phani Kiran V',
    maintainer_email='phanikiran1169@gmail.com',
    description='Eye-in-hand and eye-on-base camera calibration for Franka FR3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_marker = fr3_calibration.make_marker:main',
        ],
    },
)
