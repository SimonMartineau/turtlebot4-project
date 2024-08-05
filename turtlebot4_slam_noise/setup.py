from setuptools import find_packages, setup
import os

package_name = 'turtlebot4_slam_noise'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simon',
    maintainer_email='simon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_noise = turtlebot4_slam_noise.visual_noise:main',
            'dust_noise = turtlebot4_slam_noise.lidar_dust_noise:main'
        ],
    },
)
