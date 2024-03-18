import os
from glob import glob
from setuptools import setup

package_name = 'rosbag_time_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmilam',
    maintainer_email='tmilam@traclabs.com',
    description='Conversion tool to set playback time to cFE message time in a rosbag file',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'rosbag_time_converter = rosbag_time_converter.rosbag_time_converter:main'
            ],
    },
)
