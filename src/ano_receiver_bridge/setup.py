from glob import glob
from setuptools import find_packages, setup


package_name = 'ano_receiver_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='metro',
    maintainer_email='metro@local',
    description='Minimal ROS 2 Humble Python receiver bridge for ANO serial frames.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = ano_receiver_bridge.serial_node:main',
        ],
    },
)
