import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot4_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Martineau',
    maintainer_email='simon.martineau@ensta-bretagne.org',
    description='ROS2 custom launch scripts for starting turtlebot4 programs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dust_lidar_exp_record = turtlebot4_bringup.dust_lidar_exp_record.launch:main',
        ],
    },
)