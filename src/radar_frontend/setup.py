from glob import glob
from setuptools import find_packages, setup


package_name = 'radar_frontend'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='metro',
    maintainer_email='metro@local.com',
    description='Minimal ROS 2 lidar frontend utilities for scan quality inspection.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_scan_inspector_node = radar_frontend.radar_scan_inspector_node:main',
            'radar_scan_odometry_node = radar_frontend.radar_scan_odometry_node:main',
        ],
    },
)
